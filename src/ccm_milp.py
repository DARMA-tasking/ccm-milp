import pulp
import time
import sys
sys.path.insert(0, "../examples")

from small import SmallProblem

class Config:
    def __init__(self, is_COMCP : bool, alpha : float, beta : float, gamma : float, delta : float):
        self.is_COMCP = is_COMCP
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta

class CCM_MILP_Generator:
    def __init__(self, configuration : Config, input_problem):
        self.config = configuration

        self.rank_mems = input_problem.rank_mems
        self.rank_working_bytes = input_problem.rank_working_bytes
        self.task_loads = input_problem.task_loads
        self.task_working_bytes = input_problem.task_working_bytes
        self.task_footprint_bytes = input_problem.task_footprint_bytes
        self.task_rank = input_problem.task_rank
        self.task_id = input_problem.task_id
        self.memory_blocks = input_problem.memory_blocks
        self.memory_block_home = input_problem.memory_block_home
        self.task_memory_block_mapping = input_problem.task_memory_block_mapping
        self.task_communications = input_problem.task_communications

        self.I = len(self.rank_mems) # I is the cardinality of set R
        self.K = len(self.task_loads) # K is the cardinality of set T
        self.M = len(self.task_communications) # M is the cardinality of set C
        self.N = len(self.memory_blocks) # N is the cardinality of set S

        print(f"Total load={sum(self.task_loads)}, Mean Load={sum(self.task_loads)/self.I}")
        print(f"Ranks={self.I}, task_loads={self.K}, memory_blocks={self.N}")


    def setupMILP(self):
        # Solving a minimization of a mixed-integer linear program
        self.problem = pulp.LpProblem("CCM_MILP", pulp.LpMinimize)

        # For convenience, make these local variables
        I = self.I
        K = self.K
        M = self.M
        N = self.N

        alpha = self.config.alpha
        beta = self.config.beta
        delta = self.config.delta
        gamma = self.config.gamma

        is_COMCP = self.config.is_COMCP

        # χ: ranks <- tasks, I x K, binary variables in MILP
        χ = pulp.LpVariable.dicts("χ", ((i, k) for i in range(I) for k in range(K)), cat='Binary')

        # φ: ranks <- shared blocks, I x N, binary variables in MILP
        φ = pulp.LpVariable.dicts("φ", ((i, n) for i in range(I) for n in range(N)), cat='Binary')

        if not is_COMCP:
            # ψ: ranks <- communications, I x I x M, binary variables in MILP
            ψ = pulp.LpVariable.dicts("ψ", ((i, j, m) for i in range(I) for j in range(I) for m in range(M)), cat='Binary')

        # W_max: continuous variable in MILP for work defined by CCM model
        W_max = pulp.LpVariable("W_max", lowBound=0, cat='Continuous')

        # Add the continuous variable to the problem
        self.problem += W_max

        # Add equation 14, constraining every task to a single rank:
        for k in range(K):
            self.problem += sum(χ[i, k] for i in range(I)) == 1

        for i in range(I):
            for n in range(N):
                for p in range(len(self.task_memory_block_mapping[n])):
                    # Add equation 17
                    self.problem += φ[i, n] >= χ[i, self.task_memory_block_mapping[n][p]]

                # Add equation 18
                self.problem += φ[i, n] <= sum(χ[i, self.task_memory_block_mapping[n][p]] for p in range(len(self.task_memory_block_mapping[n])))

        for i in range(I):
            for k in range(K):
                # Add equation 19
                self.problem += (
                    sum(self.task_footprint_bytes[l] * χ[i, l] for l in range(K)) +
                    self.task_working_bytes[k] * χ[i, k] +
                    sum(self.memory_blocks[n] * φ[i, n] for n in range(N))) <= (self.rank_mems[i] - self.rank_working_bytes[i])


        if is_COMCP:
            for i in range(I):
                # Add equation 20
                self.problem += sum(self.task_loads[k] * χ[i, k] for k in range(K)) <= W_max

        self.problem.solve()

config = Config(True, 0, 0, 0, 0)
s = CCM_MILP_Generator(config, SmallProblem())
s.setupMILP()

#                           DARMA Toolkit v. 1.0.0
#
# Copyright 2024 National Technology & Engineering Solutions of Sandia, LLC
# (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S.
# Government retains certain rights in this software.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Questions? Contact darma@sandia.gov
#

import sys
import getopt
import importlib
import time

import pulp
import yaml

sys.path.insert(0, "../examples")

# Available CCM-MILP examples
avail_examples = [
    ["small", "SmallProblem"],
    ["synthetic_blocks", "SyntheticBlocks"],
    ["ccm_example_no_sub_cluster", "CCMExampleNoSubCluster"],
    ["ccm_example_with_sub_cluster", "CCMExampleWithSubCluster"]]

# Default CCM parameter values
default_parameters = {
    "alpha": 1.0,
    "beta": 0.0,
    "gamma": 0.0,
    "delta": 0.0}

def input_float(input_name: str, indent="  ", default=0.0):
    """ Interactively retrieve input with float type. """
    value = input(f"{indent}value of {input_name} [{default}]? ")
    if not value:
        return default
    try:
        value = float(value)
    except Exception as exc:
        raise TypeError("incorrect input type: type(value)") from exc
    return value

class Config:
    def __init__(self, is_FWMP: bool,
                 alpha: float, beta: float, gamma: float, delta: float,
                 use_mem_ub:bool, preserve_clusters:bool):
        print(f"\n# Initializing {'FMWP' if is_FWMP else 'COMCP'} configuration with:")
        print(f"  alpha = {alpha}")
        print(f"  beta = {beta}")
        print(f"  gamma = {gamma}")
        print(f"  delta = {delta}")
        print(f"  with{'' if use_mem_ub else 'out'} rank memory upper bound")
        self.is_FWMP = is_FWMP
        self.is_COMCP = not is_FWMP
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.use_mem_ub = use_mem_ub
        self.preserve_clusters = preserve_clusters

class CCM_MILP_Generator:
    def __init__(self, configuration : Config, input_problem):
        print(f"\n# Instantiating {type(input_problem).__name__} problem")

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
        print(f"Ranks={self.I}, task_loads={self.K}, memory_blocks={self.N} comms={self.M}")

        self.problem = None

    def setupMILP(self):
        """ Solve a minimization of a mixed-integer linear program. """
        # instantiante LP minimization problem
        self.problem = pulp.LpProblem("CCM_MILP", pulp.LpMinimize)

        # Number of ranks
        I = self.I

        # Number of tasks
        K = self.K

        # Number of communications
        M = self.M

        # Number of shared blocks
        N = self.N

        is_COMCP = self.config.is_COMCP
        is_FWMP = self.config.is_FWMP
        preserve_clusters = self.config.preserve_clusters

        # χ: ranks <- tasks, I x K, binary variables in MILP
        self.χ = pulp.LpVariable.dicts("χ", ((i, k) for i in range(I) for k in range(K)), cat='Binary')

        # φ: ranks <- shared blocks, I x N, binary variables in MILP
        self.φ = pulp.LpVariable.dicts("φ", ((i, n) for i in range(I) for n in range(N)), cat='Binary')

        # ψ: ranks <- communications I x I x M, binary variables in MILP 
        self.ψ = dict()
        if is_FWMP:
            self.ψ = pulp.LpVariable.dicts("ψ", ((i, j, m) for i in range(I) for j in range(I) for m in range(M)), cat='Binary')

        # W_max: continuous variable in MILP for work defined by CCM model
        self.W_max = pulp.LpVariable("W_max", lowBound=0, cat='Continuous')

        # Add the continuous variable to the problem
        self.problem += self.W_max

        # Add equation 14, constraining every task to a single rank:
        start_time = time.perf_counter()
        for k in range(K):
            self.problem += sum(self.χ[i, k] for i in range(I)) == 1
        end_time = time.perf_counter()
        print(f"Added basic constraints in {end_time - start_time:0.4f}s")

        # Add equations 17 and 18 for shared block constraints
        start_time = time.perf_counter()
        for i in range(I):
            for n in range(N):
                for p in range(len(self.task_memory_block_mapping[n])):
                    # Add equation 17
                    self.problem += self.φ[i, n] >= self.χ[i, self.task_memory_block_mapping[n][p]]

                # Add equation 18
                self.problem += self.φ[i, n] <= sum(self.χ[i, self.task_memory_block_mapping[n][p]] for p in range(len(self.task_memory_block_mapping[n])))
        end_time = time.perf_counter()
        print(f"Added shared blocks constraints in {end_time - start_time:0.4f}s")

        # Include memory constraints when requested
        if self.config.use_mem_ub:
            start_time = time.perf_counter()
            all_k_working_bytes_zero = True
            for i in range(K):
                if self.task_working_bytes[i] != 0:
                    all_k_working_bytes_zero = False
            for i in range(I):
                if all_k_working_bytes_zero:
                    # Add equation 19
                    self.problem += (
                        sum(self.task_footprint_bytes[l] * self.χ[i, l] for l in range(K)) +
                        sum(self.memory_blocks[n] * self.φ[i, n] for n in range(N))
                    ) <= (self.rank_mems[i] - self.rank_working_bytes[i])
                else:
                    for k in range(K):
                        # Add equation 19
                        self.problem += (
                            sum(self.task_footprint_bytes[l] * self.χ[i, l] for l in range(K)) +
                            self.task_working_bytes[k] * self.χ[i, k] +
                            sum(self.memory_blocks[n] * self.φ[i, n] for n in range(N))
                        ) <= (self.rank_mems[i] - self.rank_working_bytes[i])
            end_time = time.perf_counter()
            print(f"Added memory constraints in {end_time - start_time:0.4f}s")

        # Add communication constraints in full model case
        if is_FWMP:
            start_time = time.perf_counter()
            for i in range(I):
                for j in range(I):
                    for p in range(len(self.task_communications)):
                        # Add equation 25
                        self.problem += self.ψ[i, j, p] <= self.χ[i, self.task_communications[p][0]]
                        # Add equation 26
                        self.problem += self.ψ[i, j, p] <= self.χ[j, self.task_communications[p][1]]
                        # Add equation 27
                        self.problem += self.ψ[i, j, p] >= self.χ[i, self.task_communications[p][0]] + self.χ[j, self.task_communications[p][1]] - 1
            end_time = time.perf_counter()
            print(f"Added communications constraints in {end_time - start_time:0.4f}s")

        # Add continuous constraints
        start_time = time.perf_counter()
        if is_COMCP:
            for i in range(I):
                # Add equation 20
                self.problem += sum(self.task_loads[k] * self.χ[i, k] for k in range(K)) <= self.W_max
        elif is_FWMP:
            for i in range(I):
                # For rank i, build a list of all the remote shared blocks for the forth term of equation 30
                remote_blocks = [n for n in range(N) if self.memory_block_home[n] != i]

                # For rank i, build a list of all the other machines (all but i) for the second term of equation 30
                other_machines = [j for j in range(I) if j != i]

                # Compute unchanging terms in equation 30
                alpha_term = self.config.alpha * sum(
                    self.task_loads[k] * self.χ[i, k] for k in range(K))
                gamma_term = self.config.gamma * sum(
                    self.ψ[i, i, p] * self.task_communications[p][2]
                    for p in range(len(self.task_communications)))
                delta_term = self.config.delta * sum(
                    self.memory_blocks[remote_blocks[p]] * self.φ[i, remote_blocks[p]]
                    for p in range(len(remote_blocks)))

                # Add equation 30 for first transposition of beta term (σ(i,j) = {i,j})
                self.problem += alpha_term + gamma_term + delta_term + self.config.beta * sum(
                    self.ψ[i, j, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= self.W_max

                # Add equation 30 for second transposition of beta term (σ(i,j) = {j,i})
                self.problem += alpha_term + gamma_term + delta_term + self.config.beta * sum(
                    self.ψ[j, i, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= self.W_max

            if preserve_clusters:
                end_time = time.perf_counter()
                # Add cluster-preserving constraints
                for n in range(N):
                    self.problem += sum(self.φ[i, n] for i in range(I)) == 1
                print(f"Added cluster preserving constraints in {end_time - start_time:0.4f}s")
                start_time = time.perf_counter()
        end_time = time.perf_counter()
        print(f"Added continuous constraints in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

    def outputSolution(self):
        if self.problem.status == pulp.LpStatusOptimal:
            solution = {"W_max": pulp.value(self.W_max)}
            machine_memory_blocks_assigned = [[] for i in range(self.I)]
            rank_totals = {}
            total_unhomed_blocks = 0

            # Iterate over ranks
            for i in range(self.I):
                unhomed_blocks = 0
                delta_cost = 0
                for n in range(self.N):
                    if pulp.value(self.φ[i, n]) == 1:
                        machine_memory_blocks_assigned[i].append(n)
                        if i != self.memory_block_home[n]:
                            unhomed_blocks += 1
                            delta_cost += self.memory_blocks[n] * self.config.delta
                total_unhomed_blocks += unhomed_blocks
                total_load = 0.0
                total_work = delta_cost
                for k in range(self.K):
                    if pulp.value(self.χ[i, k]) == 1:
                        solution[f"Task {k} of load {self.task_loads[k]} and memory blocks {machine_memory_blocks_assigned[i]} assigned to rank {i}"] = True
                        total_load += self.task_loads[k]
                        total_work += self.config.alpha * self.task_loads[k]

                if self.ψ:
                    comm_cost = 0.0
                    for j in range(self.I):
                        for m in range(self.M):
                            if pulp.value(self.ψ[i, j, m]) == 1.0 and pulp.value(self.χ[i, self.task_communications[m][0]]) == 1.0 and pulp.value(self.χ[j, self.task_communications[m][1]]) == 1.0:
                                comm_cost += self.task_communications[m][2] * (
                                    self.config.gamma if i == j else self.config.beta)
                    total_work += comm_cost

                # Keep track of totals on rank
                rank_totals[i] = (total_load, total_work, unhomed_blocks)

            # Compute assignment array of tasks to ranks 
            assignments = [-1] * self.K
            for i in range(self.I):
                for k in range(self.K):
                    # Some solvers do not output a solution that is exactly 1 albeit binary
                    if pulp.value(self.χ[i, k]) > 0.5:
                        assignments[k] = i
            
            print("\n# Detailed solution:")
            for key, value in solution.items():
                if key == "W_max":
                    continue
                elif value:
                    print(key)

            print("\n# Solution summary:")
            for i in range(self.I):
                t = rank_totals[i]
                print(f"Rank {i}: L = {t[0]}, W = {t[1]}, unhomed: {t[2]}")
            print("W_max =", solution["W_max"])
            print(f"$assignments={assignments};")

        else: # if self.problem.status == pulp.LpStatusOptimal
            print("# No solution found")

def run_interactive():
    # Build example
    print("# Available examples:")
    for i, [file_name, class_name] in enumerate(avail_examples):
        print(f"  {i}) {file_name}.{class_name}")
    example_id = int(input("  example index: "))
    if example_id not in range(len(avail_examples)):
        raise ValueError(f"unvailable example index: {example_id}")

    # Interactively get and return problem configuration
    print("\n# Model configuration:")
    return [
        avail_examples[example_id],
        (is_fwmp := (input("  FWMP [y/N]? ") == 'y')),
        input_float("alpha") if is_fwmp else default_parameters["alpha"],
        input_float("beta") if is_fwmp else default_parameters["beta"],
        input_float("gamma") if is_fwmp else default_parameters["gamma"],
        input_float("delta") if is_fwmp else default_parameters["delta"],
        input("  bounded memory [y/N]? ") == 'y',
        input("  preserve clusters [y/N]? ") == 'y']

def run_batch(file_name: str):
    # Try to read YAML configuration file
    parameters = {}
    try:
        with open(file_name, "r", encoding="utf-8") as f:
            parameters = yaml.safe_load(f)
    except: # pylint: disable=bare-except
        print ("*** Could not parse", file_name)
        sys.exit(1)

    # Parse parameters
    print("# Model configuration:")
    c_float, c_bool, ccm_example = {}, {}, None
    for k, v in parameters.items():
        print(f"  {k}: {v}")
        if k in ("alpha", "beta", "gamma", "delta"):
            c_float[k] = float(v)
        elif k in ("is_fwmp", "bounded_memory", "preserve_clusters"):
            c_bool[k] = bool(v)
        elif k == "example_id":
            ccm_example = avail_examples[int(v)]
        else:
            print ("*** Incorrect key:", k)
            sys.exit(1)

    # Retrieve and return problem configuration
    if not ccm_example:
        print ("*** No CCM example was defined")
        sys.exit(1)
    return [
        ccm_example,
        c_bool.get("is_fwmp", False),
        c_float.get("alpha", default_parameters["alpha"]),
        c_float.get("beta", default_parameters["beta"]),
        c_float.get("gamma", default_parameters["gamma"]),
        c_float.get("delta", default_parameters["delta"]),
        c_bool.get("bounded_memory", False),
        c_bool.get("preserve_clusters", False)]

def main(argv):
    # Parse possible command-line arguments
    try:
        opts, _ = getopt.getopt(argv,"c:")
    except getopt.GetoptError:
        print ("*** Usage:ccm_milp.py [-c <YAML configuration file>]")
        sys.exit(1)

    # Default execution mode is interactive
    file_name = None
    for o, a in opts:
        if o == '-c':
            file_name = a

    # Retrieve parameters either interactively or in batch mode
    ccm_problem = None
    if file_name:
        ccm_example, fwmp, alpha, beta, gamma, delta, bnd_mem, pr_cl = run_batch(file_name)
    else:
        ccm_example, fwmp, alpha, beta, gamma, delta, bnd_mem, pr_cl  = run_interactive()

    # Build and save linear program
    ccm_problem = CCM_MILP_Generator(
        Config(fwmp, alpha, beta, gamma, delta, bnd_mem, pr_cl),
        getattr(importlib.import_module(ccm_example[0]), ccm_example[1])())
    ccm_problem.setupMILP()
    ccm_problem.problem.writeLP("problem.lp")

    # Solve linear program
    ccm_problem.problem.solve()

    # Report solution to linear program when found
    ccm_problem.outputSolution()
    print()
    
if __name__ == "__main__":
    main(sys.argv[1:])

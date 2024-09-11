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

import time
import os
import pulp

from ccm_milp.configuration import Config

class CcmMilpGenerator:
    """Manage CCM-MILP problem - Setup, Generate and Solve"""

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

        self.i = len(self.rank_mems) # i is the cardinality of set R
        self.k = len(self.task_loads) # k is the cardinality of set T
        self.m = len(self.task_communications) # m is the cardinality of set C
        self.n = len(self.memory_blocks) # n is the cardinality of set S

        self.chi = None
        self.phi = None
        self.psi = None
        self.w_max = None

        print(f"Total load={sum(self.task_loads)}, Mean Load={sum(self.task_loads)/self.i}")
        print(f"Ranks={self.i}, task_loads={self.k}, memory_blocks={self.n} comms={self.m}")

        self.problem = None

    def generate_problem(self):
        """Generate the problem"""
        self.setup_milp()
        self.write_lp_to_file(os.path.join(os.path.dirname(__file__),'..', 'problem.lp'))
        self.write_mps_to_file(os.path.join(os.path.dirname(__file__),'..', 'problem.mps'))

    def generate_problem_and_solve(self, solver_name: str):
        """Generate the problem and solve it"""
        # Call launch
        self.generate_problem()

        # Check solver
        if solver_name not in pulp.listSolvers(onlyAvailable=True):
            print("*** Available LP solvers: ", pulp.listSolvers(onlyAvailable=True))
            raise ValueError(f"Solver not found: {solver_name}")

        print("# Solver: ", solver_name)
        self.solve_problem(self.problem, solver_name)

    def output_solution(self):
        """Generate output report"""
        if self.problem.status == pulp.LpStatusOptimal:
            solution = {"w_max": pulp.value(self.w_max)}
            machine_memory_blocks_assigned = [[] for i in range(self.i)]
            rank_totals = {}
            total_unhomed_blocks = 0

            # Iterate over ranks
            for i in range(self.i):
                unhomed_blocks = 0
                delta_cost = 0
                for n in range(self.n):
                    if pulp.value(self.phi[i, n]) == 1:
                        machine_memory_blocks_assigned[i].append(n)
                        if i != self.memory_block_home[n]:
                            unhomed_blocks += 1
                            delta_cost += self.memory_blocks[n] * self.config.delta
                total_unhomed_blocks += unhomed_blocks
                total_load = 0.0
                total_work = delta_cost
                for k in range(self.k):
                    if pulp.value(self.chi[i, k]) == 1:
                        solution[f"Task {k} of load {self.task_loads[k]}"
                            f" and memory blocks {machine_memory_blocks_assigned[i]} assigned to rank {i}"
                        ] = True
                        total_load += self.task_loads[k]
                        total_work += self.config.alpha * self.task_loads[k]

                if self.psi:
                    comm_cost = 0.0
                    for j in range(self.i):
                        for m in range(self.m):
                            if( pulp.value(self.psi[i, j, m]) == 1.0 and
                                pulp.value(self.chi[i, self.task_communications[m][0]]) == 1.0 and
                                pulp.value(self.chi[j, self.task_communications[m][1]]) == 1.0
                            ):
                                comm_cost += self.task_communications[m][2] * (
                                    self.config.gamma if i == j else self.config.beta
                                )
                    total_work += comm_cost

                # Keep track of totals on rank
                rank_totals[i] = (total_load, total_work, unhomed_blocks)

            # Compute assignment array of tasks to ranks
            assignments = [-1] * self.k
            for i in range(self.i):
                for k in range(self.k):
                    # Some solvers do not output a solution that is exactly 1 albeit binary
                    if pulp.value(self.chi[i, k]) > 0.5:
                        assignments[k] = i

            print("\n# Detailed solution:")
            for key, value in solution.items():
                if key == "w_max":
                    continue
                elif value:
                    print(key)

            print("\n# Solution summary:")
            for i in range(self.i):
                t = rank_totals[i]
                print(f"Rank {i}: L = {t[0]}, W = {t[1]}, unhomed: {t[2]}")
            print("w_max =", solution["w_max"])
            print(f"$assignments={assignments};")

        else: # if self.problem.status == pulp.LpStatusOptimal
            print("# No solution found")

    def setup_milp(self):
        """ Solve a minimization of a mixed-integer linear program. """
        # instantiante LP minimization problem
        self.problem = pulp.LpProblem("CCM_MILP", pulp.LpMinimize)

        # Number of ranks
        self_i = self.i

        # Number of tasks
        self_k = self.k

        # Number of communications
        self_m = self.m

        # Number of shared blocks
        self_n = self.n

        is_comcp = self.config.is_comcp
        is_fmwp = self.config.is_fmwp

        # chi: ranks <- tasks, self_i x self_k, binary variables in MILP
        self.chi = pulp.LpVariable.dicts("chi", ((i, k) for i in range(self_i) for k in range(self_k)), cat='Binary')

        # φ: ranks <- shared blocks, self_i x self_n, binary variables in MILP
        self.phi = pulp.LpVariable.dicts("phi", ((i, n) for i in range(self_i) for n in range(self_n)), cat='Binary')

        # ψ: ranks <- communications self_i x self_i x self_m, binary variables in MILP
        self.psi = {}
        if is_fmwp:
            self.psi = pulp.LpVariable.dicts("psi", ((i, j, m)
                for i in range(self_i)
                for j in range(self_i)
                for m in range(self_m)
            ), cat='Binary')

        # w_max: continuous variable in MILP for work defined by CCM model
        self.w_max = pulp.LpVariable("w_max", lowBound=0, cat='Continuous')

        # Add the continuous variable to the problem
        self.problem += self.w_max

        # Add equation 14, constraining every task to a single rank:
        start_time = time.perf_counter()
        for k in range(self_k):
            self.problem += sum(self.chi[i, k] for i in range(self_i)) == 1
        end_time = time.perf_counter()
        print(f"Added basic constraints in {end_time - start_time:0.4f}s")

        # Add equations 17 and 18 for shared block constraints
        start_time = time.perf_counter()
        for i in range(self_i):
            for n in range(self_n):
                for p in range(len(self.task_memory_block_mapping[n])):
                    # Add equation 17
                    self.problem += self.phi[i, n] >= self.chi[i, self.task_memory_block_mapping[n][p]]

                # Add equation 18
                self.problem += self.phi[i, n] <= sum(
                    self.chi[i, self.task_memory_block_mapping[n][p]]
                    for p in range(len(self.task_memory_block_mapping[n]))
                )
        end_time = time.perf_counter()
        print(f"Added shared blocks constraints in {end_time - start_time:0.4f}s")

        # Include memory constraints when requested
        if self.config.bounded_memory:
            start_time = time.perf_counter()
            all_k_working_bytes_zero = True
            for i in range(self_k):
                if self.task_working_bytes[i] != 0:
                    all_k_working_bytes_zero = False
            for i in range(self_i):
                if all_k_working_bytes_zero:
                    # Add equation 19
                    self.problem += (
                        sum(self.task_footprint_bytes[l] * self.chi[i, l] for l in range(self_k)) +
                        sum(self.memory_blocks[n] * self.phi[i, n] for n in range(self_n))
                    ) <= (self.rank_mems[i] - self.rank_working_bytes[i])
                else:
                    for k in range(self_k):
                        # Add equation 19
                        self.problem += (
                            sum(self.task_footprint_bytes[l] * self.chi[i, l] for l in range(self_k)) +
                            self.task_working_bytes[k] * self.chi[i, k] +
                            sum(self.memory_blocks[n] * self.phi[i, n] for n in range(self_n))
                        ) <= (self.rank_mems[i] - self.rank_working_bytes[i])
            end_time = time.perf_counter()
            print(f"Added memory constraints in {end_time - start_time:0.4f}s")

        # Add communication constraints in full model case
        if is_fmwp:
            start_time = time.perf_counter()
            for i in range(self_i):
                for j in range(self_i):
                    for p,_ in enumerate(self.task_communications): #for p in range(len(self.task_communications)):
                        # Add equation 25
                        self.problem += self.psi[i, j, p] <= self.chi[i, self.task_communications[p][0]]
                        # Add equation 26
                        self.problem += self.psi[i, j, p] <= self.chi[j, self.task_communications[p][1]]
                        # Add equation 27
                        self.problem += (
                              self.psi[i, j, p] >= self.chi[i, self.task_communications[p][0]]
                            + self.chi[j, self.task_communications[p][1]] - 1
                        )
            end_time = time.perf_counter()
            print(f"Added communications constraints in {end_time - start_time:0.4f}s")

        # Add continuous constraints
        start_time = time.perf_counter()
        if is_comcp:
            for i in range(self_i):
                # Add equation 20
                self.problem += sum(self.task_loads[k] * self.chi[i, k] for k in range(self_k)) <= self.w_max
        elif is_fmwp:
            for i in range(self_i):
                # For rank i, build a list of all the remote shared blocks for the forth term of equation 30
                remote_blocks = [n for n in range(self_n) if self.memory_block_home[n] != i]

                # For rank i, build a list of all the other machines (all but i) for the second term of equation 30
                other_machines = [j for j in range(self_i) if j != i]

                # Compute unchanging terms in equation 30
                alpha_term = self.config.alpha * sum(
                    self.task_loads[k] * self.chi[i, k] for k in range(self_k))
                gamma_term = self.config.gamma * sum(
                    self.psi[i, i, p] * self.task_communications[p][2]
                    for p in range(len(self.task_communications)))
                delta_term = self.config.delta * sum(
                    self.memory_blocks[remote_blocks[p]] * self.phi[i, remote_blocks[p]]
                    for p in range(len(remote_blocks)))

                # Add equation 30 for first transposition of beta term (σ(i,j) = {i,j})
                self.problem += alpha_term + gamma_term + delta_term + self.config.beta * sum(
                    self.psi[i, j, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= self.w_max

                # Add equation 30 for second transposition of beta term (σ(i,j) = {j,i})
                self.problem += alpha_term + gamma_term + delta_term + self.config.beta * sum(
                    self.psi[j, i, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= self.w_max
        end_time = time.perf_counter()
        print(f"Added continuous constraints in {end_time - start_time:0.4f}s")

        # Add cluster-preserving constraints when requested
        if self.config.preserve_clusters:
            start_time = time.perf_counter()
            for n in range(self_n):
                self.problem += sum(self.phi[i, n] for i in range(self_i)) == 1
            end_time = time.perf_counter()
            print(f"Added cluster-preserving constraints in {end_time - start_time:0.4f}s")

    def write_lp_to_file(self, file_name : str):
        """Generate the problem file .pl"""
        self.problem.writeLP(file_name)

    def write_mps_to_file(self, file_name : str):
        """Generate the problem file .mps"""
        self.problem.writeMPS(file_name)

    @staticmethod
    def solve_problem(problem, solver_name: str):
        """Solve the given problem"""

        # Solver
        solver = pulp.getSolver(solver=solver_name, keepFiles=True)

        # Set solver
        problem.setSolver(solver)

        # Solve the problem
        problem.solve()

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

from CcmMilp.Configuration import Config

class CcmMilpGenerator:
    def __init__(self, configuration : Config, input_problem):
        print(f"# Instantiating {type(input_problem).__name__} problem")

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

    def setup_milp(self):
        """ Solve a minimization of a mixed-integer linear program. """
        # instantiante LP minimization problem
        self.problem = pulp.LpProblem("CCM_MILP", pulp.LpMinimize)

        # For convenience, make these local variables
        self_i = self.i
        self_k = self.k
        self_m = self.m
        self_n = self.n

        alpha = self.config.alpha
        beta = self.config.beta
        delta = self.config.delta
        gamma = self.config.gamma

        is_comp = self.config.is_comcp
        is_fwmp = self.config.is_fwmp
        preserve_clusters = self.config.preserve_clusters

        # chi: ranks <- tasks, self_i x self_k, binary variables in MILP
        chi = pulp.LpVariable.dicts("chi", ((i, k) for i in range(self_i) for k in range(self_k)), cat='Binary')

        # phi: ranks <- shared blocks, self_i x self_n, binary variables in MILP
        phi = pulp.LpVariable.dicts("phi", ((i, n) for i in range(self_i) for n in range(self_n)), cat='Binary')

        psi = dict()
        if is_fwmp:
            # psi: ranks <- communications, self_i x self_i x self_m, binary variables in MILP
            psi = pulp.LpVariable.dicts(
                "psi", ((i, j, m) for i in range(self_i) for j in range(self_i) for m in range(self_m)), 
                cat='Binary'
            )

        # w_max: continuous variable in MILP for work defined by CCM model
        w_max = pulp.LpVariable("w_max", lowBound=0, cat='Continuous')

        # Add the continuous variable to the problem
        self.problem += w_max

        start_time = time.perf_counter()

        # Add equation 14, constraining every task to a single rank:
        for k in range(self_k):
            self.problem += sum(chi[i, k] for i in range(self_i)) == 1

        end_time = time.perf_counter()
        print(f"Added basic constraint in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

        for i in range(self_i):
            for n in range(self_n):
                for p in range(len(self.task_memory_block_mapping[n])):
                    # Add equation 17
                    self.problem += phi[i, n] >= chi[i, self.task_memory_block_mapping[n][p]]

                # Add equation 18
                self.problem += phi[i, n] <= sum(
                    chi[i, self.task_memory_block_mapping[n][p]] for p in range(len(self.task_memory_block_mapping[n]))
                )

        end_time = time.perf_counter()
        print(f"Added shared blocks constraint in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

        all_k_working_bytes_zero = True
        for i in range(self_k):
            if self.task_working_bytes[i] != 0:
                all_k_working_bytes_zero = False

        # Include memory constraints when requested
        if self.config.use_mem_ub:
            for i in range(self_i):
                if all_k_working_bytes_zero:
                    # Add equation 19
                    self.problem += (
                        sum(self.task_footprint_bytes[l] * chi[i, l] for l in range(self_k)) +
                        sum(
                                self.memory_blocks[n]
                            *   phi[i, n] for n in range(self_n))) <= (self.rank_mems[i] - self.rank_working_bytes[i]
                        )
                else:
                    for k in range(self_k):
                        # Add equation 19
                        self.problem += (
                            sum(
                                    self.task_footprint_bytes[l]
                                *   chi[i, l] for l in range(self_k)
                            ) +
                            self.task_working_bytes[k] * chi[i, k] +
                            sum(
                                    self.memory_blocks[n]
                                *   phi[i, n] for n in range(self_n)
                            )
                        ) <= (self.rank_mems[i] - self.rank_working_bytes[i])

            end_time = time.perf_counter()
            print(f"Added memory constraints in {end_time - start_time:0.4f}s")
            start_time = time.perf_counter()

        if is_fwmp:
            for i in range(self_i):
                for j in range(self_i):
                    # plynt error: p in range(len(self.task_communications))
                    for p, _ in enumerate(self.task_communications):
                        # Add equation 25
                        self.problem += psi[i, j, p] <= chi[i, self.task_communications[p][0]]
                        # Add equation 26
                        self.problem += psi[i, j, p] <= chi[j, self.task_communications[p][1]]
                        # Add equation 27
                        self.problem += (
                                psi[i, j, p] >= chi[i, self.task_communications[p][0]]
                            +   chi[j, self.task_communications[p][1]] - 1
                        )

        end_time = time.perf_counter()
        if is_fwmp:
            print(f"Added comm constraints in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

        if is_comp:
            for i in range(self_i):
                # Add equation 20
                self.problem += sum(self.task_loads[k] * chi[i, k] for k in range(self_k)) <= w_max

        if is_fwmp:
            for i in range(self_i):
                # For rank i, build a list of all the remote shared blocks for the forth term of equation 30
                remote_blocks = [n for n in range(self_n) if self.memory_block_home[n] != i]

                # For rank i, build a list of all the other machines (all but i) for the second term of equation 30
                other_machines = [j for j in range(self_i) if j != i]

                # Compute unchanging terms in equation 30
                alpha_term = sum(
                        self.task_loads[k]
                    *   chi[i, k]
                    *   alpha for k in range(self_k)
                )
                gamma_term = sum(
                        gamma
                    *   psi[i, i, p]
                    *   self.task_communications[p][2] for p in range(len(self.task_communications))
                )
                delta_term = sum(
                        self.memory_blocks[remote_blocks[p]]
                    *   phi[i, remote_blocks[p]]
                    *   delta for p in range(len(remote_blocks))
                )

                # Add equation 30 for first transposition of beta term (σ(i,j) = {i,j})
                self.problem += alpha_term + gamma_term + delta_term + sum(
                    beta * psi[i, j, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= w_max

                # Add equation 30 for second transposition of beta term (σ(i,j) = {j,i})
                self.problem += alpha_term + gamma_term + delta_term + sum(
                    beta * psi[j, i, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= w_max

            if preserve_clusters:
                end_time = time.perf_counter()
                # Add cluster-preserving constraints
                for n in range(self_n):
                    self.problem += sum(phi[i, n] for i in range(self_i)) == 1
                print(f"Added cluster preserving constraints in {end_time - start_time:0.4f}s")
                start_time = time.perf_counter()

        end_time = time.perf_counter()
        print(f"Added continuous constraints in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

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

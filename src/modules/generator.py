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

from modules.configuration import Config

class CCM_MILP_Generator:
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
        is_FWMP = self.config.is_FWMP
        preserve_clusters = self.config.preserve_clusters

        # χ: ranks <- tasks, I x K, binary variables in MILP
        χ = pulp.LpVariable.dicts("chi", ((i, k) for i in range(I) for k in range(K)), cat='Binary')

        # φ: ranks <- shared blocks, I x N, binary variables in MILP
        φ = pulp.LpVariable.dicts("phi", ((i, n) for i in range(I) for n in range(N)), cat='Binary')

        ψ = dict()
        if is_FWMP:
            # ψ: ranks <- communications, I x I x M, binary variables in MILP
            ψ = pulp.LpVariable.dicts("psi", ((i, j, m) for i in range(I) for j in range(I) for m in range(M)), cat='Binary')

        # w_max: continuous variable in MILP for work defined by CCM model
        w_max = pulp.LpVariable("w_max", lowBound=0, cat='Continuous')

        # Add the continuous variable to the problem
        self.problem += w_max

        start_time = time.perf_counter()

        # Add equation 14, constraining every task to a single rank:
        for k in range(K):
            self.problem += sum(χ[i, k] for i in range(I)) == 1

        end_time = time.perf_counter()
        print(f"Added basic constraint in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

        for i in range(I):
            for n in range(N):
                for p in range(len(self.task_memory_block_mapping[n])):
                    # Add equation 17
                    self.problem += φ[i, n] >= χ[i, self.task_memory_block_mapping[n][p]]

                # Add equation 18
                self.problem += φ[i, n] <= sum(χ[i, self.task_memory_block_mapping[n][p]] for p in range(len(self.task_memory_block_mapping[n])))

        end_time = time.perf_counter()
        print(f"Added shared blocks constraint in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

        all_k_working_bytes_zero = True
        for i in range(K):
            if self.task_working_bytes[i] != 0:
                all_k_working_bytes_zero = False

        # Include memory constraints when requested
        if self.config.use_mem_ub:
            for i in range(I):
                if all_k_working_bytes_zero:
                    # Add equation 19
                    self.problem += (
                        sum(self.task_footprint_bytes[l] * χ[i, l] for l in range(K)) +
                        sum(self.memory_blocks[n] * φ[i, n] for n in range(N))) <= (self.rank_mems[i] - self.rank_working_bytes[i])
                else:
                    for k in range(K):
                        # Add equation 19
                        self.problem += (
                            sum(self.task_footprint_bytes[l] * χ[i, l] for l in range(K)) +
                            self.task_working_bytes[k] * χ[i, k] +
                            sum(self.memory_blocks[n] * φ[i, n] for n in range(N))) <= (self.rank_mems[i] - self.rank_working_bytes[i])

            end_time = time.perf_counter()
            print(f"Added memory constraints in {end_time - start_time:0.4f}s")
            start_time = time.perf_counter()

        if is_FWMP:
            for i in range(I):
                for j in range(I):
                    for p in range(len(self.task_communications)):
                        # Add equation 25
                        self.problem += ψ[i, j, p] <= χ[i, self.task_communications[p][0]]
                        # Add equation 26
                        self.problem += ψ[i, j, p] <= χ[j, self.task_communications[p][1]]
                        # Add equation 27
                        self.problem += ψ[i, j, p] >= χ[i, self.task_communications[p][0]] + χ[j, self.task_communications[p][1]] - 1

        end_time = time.perf_counter()
        if is_FWMP:
            print(f"Added comm constraints in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

        if is_COMCP:
            for i in range(I):
                # Add equation 20
                self.problem += sum(self.task_loads[k] * χ[i, k] for k in range(K)) <= w_max

        if is_FWMP:
            for i in range(I):
                # For rank i, build a list of all the remote shared blocks for the forth term of equation 30
                remote_blocks = [n for n in range(N) if self.memory_block_home[n] != i]

                # For rank i, build a list of all the other machines (all but i) for the second term of equation 30
                other_machines = [j for j in range(I) if j != i]

                # Compute unchanging terms in equation 30
                alpha_term = sum(self.task_loads[k] * χ[i, k] * alpha for k in range(K))
                gamma_term = sum(gamma * ψ[i, i, p] * self.task_communications[p][2] for p in range(len(self.task_communications)))
                delta_term = sum(self.memory_blocks[remote_blocks[p]] * φ[i, remote_blocks[p]] * delta for p in range(len(remote_blocks)))

                # Add equation 30 for first transposition of beta term (σ(i,j) = {i,j})
                self.problem += alpha_term + gamma_term + delta_term + sum(
                    beta * ψ[i, j, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= w_max

                # Add equation 30 for second transposition of beta term (σ(i,j) = {j,i})
                self.problem += alpha_term + gamma_term + delta_term + sum(
                    beta * ψ[j, i, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= w_max

            if preserve_clusters:
                end_time = time.perf_counter()
                # Add cluster-preserving constraints
                for n in range(N):
                    self.problem += sum(φ[i, n] for i in range(I)) == 1
                print(f"Added cluster preserving constraints in {end_time - start_time:0.4f}s")
                start_time = time.perf_counter()

        end_time = time.perf_counter()
        print(f"Added continuous constraints in {end_time - start_time:0.4f}s")
        start_time = time.perf_counter()

    def writeLPToFile(self, file_name : str):
        self.problem.writeLP(file_name)
        
    def writeMpsToFile(self, file_name : str):
        self.problem.writeMPS(file_name)

    def launch(self):
        self.setupMILP()
        self.writeLPToFile(os.path.join(os.path.dirname(__file__),'..', 'problem.lp'))
        self.writeMpsToFile(os.path.join(os.path.dirname(__file__),'..', 'problem.mps'))
            
    def launchAndSolve(self, solverName: str):
        # Call launch         
        self.launch()

        # Check solver         
        if (solverName not in pulp.listSolvers(onlyAvailable=True)):
            print("*** Available LP solvers: ", pulp.listSolvers(onlyAvailable=True))
            raise ValueError(f"Solver not found: {solverName}")
        else: 
            print("# Solver: ", solverName)


        # Solver 
        solver = pulp.getSolver(solver=solverName, keepFiles=True)

        # Set solver
        self.problem.setSolver(solver)

        # Solve the problem 
        self.problem.solve()
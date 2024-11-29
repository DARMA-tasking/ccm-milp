#                           DARMA Toolkit v. 1.5.0
#
# Copyright 2019-2024 National Technology & Engineering Solutions of Sandia, LLC
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
import sys
import json
import typing
import math
import pulp
from .configuration import Configuration, DefaultParameters
from .data import Data

# Default CCM parameter values
DP: typing.Final = DefaultParameters()

class Generator:
    """Manage CCM-MILP problem: setup, generate, and solve"""

    def __init__(self, config: Configuration, input_problem):
        print(f"\n# Generating {type(input_problem).__name__} linear problem")
        # Keep track of configuration
        self.config = config

        # Rank parameters
        self.rank_M_inf = input_problem.rank_mems
        self.node_M_inf = input_problem.node_mems
        self.I = len(self.rank_M_inf)
        print(f"  I = {self.I} ranks")
        self.rank_M_baseline = input_problem.rank_working_bytes

        # Task parameters
        self.task_loads = input_problem.task_loads
        self.K = len(self.task_loads)
        print(f"  K = {self.K} tasks")
        sum_loads = sum(self.task_loads)
        print(f"  total rank load = {sum_loads}, average rank load = {sum_loads/self.I}")
        self.task_M_overhead = input_problem.task_working_bytes
        self.task_M_baseline = input_problem.task_footprint_bytes
        self.task_rank = input_problem.task_rank
        self.task_id = input_problem.task_id

        # Communication parameters
        self.task_communications = input_problem.task_communications
        self.M = len(self.task_communications)
        print(f"  M = {self.M} communications")

        # Shared memory block parameters
        self.memory_blocks = input_problem.memory_blocks
        self.N = len(self.memory_blocks)
        print(f"  N = {self.N} shared memory blocks")
        self.memory_block_home = input_problem.memory_block_home
        self.task_memory_block_mapping = input_problem.task_memory_block_mapping

        # Check consistency of rank per node parameters
        self.Q = self.config.ranks_per_node
        if self.I % self.Q:
            print(f"*** ERROR: number of ranks per node {self.Q} does not divide number of ranks {self.I}")
            sys.exit(1)
        print(f"  Q = {self.Q} rank{'s' if self.Q > 1 else ''} per node (i.e. {self.I // self.Q} nodes)")

        # Tasks to ranks assignment matrix
        self.chi = None

        # Shared blocks to ranks assignment matrix
        self.phi = None

        # Shared ranks to communications assignment matrix
        self.psi = None

        # Maximum work continuous variable
        self.w_max = None

        # The linear problem
        self.problem = None

    def generate_problem(self):
        """Generate the problem"""
        self.setup_milp()
        self.write_lp_to_file(os.path.join(os.path.dirname(__file__),"..", "problem.lp"))
        self.write_mps_to_file(os.path.join(os.path.dirname(__file__),"..", "problem.mps"))

    def generate_problem_and_solve(self, solver_name: str):
        """Generate the problem and solve it"""
        # Call launch
        self.generate_problem()

        # Check solver
        if solver_name not in pulp.listSolvers(onlyAvailable=True):
            print("# Available LP solvers: ", pulp.listSolvers(onlyAvailable=True))
            raise ValueError(f"Solver not found: {solver_name}")

        # Execute solver
        print("\n# Solver:", solver_name)
        self.solve_problem(self.problem, solver_name)

    def verify_and_tally_edge(self, r_snd: int, r_rcv: int, c_ind: int):
        """Check psi-chi consistency and return weight of existing edges"""
        if not pulp.value(self.psi[r_snd, r_rcv, c_ind]):
            # Edges absent from psi do not exist
            return 0.0

        # Retrieve communication entry
        comm = self.task_communications[c_ind]

        # Perform sanity checks
        if not pulp.value(self.chi[r_snd, comm[0]]):
            raise ValueError(
                f"Inconsistent results: communication edge {c_ind}"
                f" initiating from rank {r_snd}"
                f" but its starting point {comm[0]}"
                f" does not belong to rank {r_snd}")
        if not pulp.value(self.chi[r_rcv, comm[1]]):
            raise ValueError(
                f"Inconsistent results: communication edge {c_ind}"
                f" terminating at rank {r_rcv}"
                f" but its end point {comm[1]}"
                f" does not belong to rank {r_rcv}")

        # Return communication weight
        return comm[2]

    def output_solution(self, permutation_file: str = "permutation.json"):
        """Generate output report"""
        if self.problem.status == pulp.LpStatusOptimal:
            solution = {"w_max": pulp.value(self.w_max)}
            machine_memory_blocks_assigned = [[] for i in range(self.I)]
            rank_totals = []
            total_unhomed_blocks = 0

            # Iterate over ranks
            for i in range(self.I):
                # Compute work for rank i
                unhomed_blocks = 0
                delta_cost = 0
                for n in range(self.N):
                    if pulp.value(self.phi[i, n]) == 1:
                        machine_memory_blocks_assigned[i].append(n)
                        if i != self.memory_block_home[n]:
                            unhomed_blocks += 1
                            delta_cost += self.memory_blocks[n] * self.config.delta
                total_unhomed_blocks += unhomed_blocks
                total_load = 0.0
                total_work = delta_cost
                for k in range(self.K):
                    if pulp.value(self.chi[i, k]) == 1:
                        solution[f"Task {k} of load {self.task_loads[k]}"
                            f" and memory blocks {machine_memory_blocks_assigned[i]} assigned to rank {i}"
                        ] = True
                        total_load += self.task_loads[k]
                        total_work += self.config.alpha * self.task_loads[k]

                # Add communication costs when relevant
                if self.psi:
                    c_l, c_o, c_i = 0.0, 0.0, 0.0
                    # Iterate over ranks
                    for j in range(self.I):
                        # Check for communications between ranks i and j
                        for m in range(self.M):
                            # Distinguish local from global communications
                            if i == j:
                                # Tally local communications
                                c_l += self.verify_and_tally_edge(i, i, m)

                                # Skip subsequent non-local communications operations
                                continue

                            # Tally outgoing communications
                            c_o += self.verify_and_tally_edge(i, j, m)

                            # Tally incoming communications
                            c_i += self.verify_and_tally_edge(j, i, m)

                    # Update rank total work with communication costs
                    total_work += self.config.beta * max(c_o, c_i) + self.config.gamma * c_l

                # Keep track of totals on rank
                rank_totals.append((total_load, total_work, unhomed_blocks))

            # Compute assignment array of tasks to ranks
            assignments = [-1] * self.K
            for i in range(self.I):
                for k in range(self.K):
                    # Some solvers do not output a solution that is exactly 1 albeit binary
                    if pulp.value(self.chi[i, k]) > 0.5:
                        assignments[k] = i

            # Create permutation file
            with open(os.path.join(self.output_dir(), permutation_file), 'w', encoding="utf-8") as f:
                json.dump(assignments, f)

            print("\n# Detailed solution:")
            for key, value in solution.items():
                if key == "w_max":
                    continue
                if value:
                    print(f"  {key}")

            print("\n# Solution summary:")
            for i in range(self.I):
                print(f"  Rank {i}: "
                      f"L = {rank_totals[i][0]}, "
                      f"W = {rank_totals[i][1]}, "
                      f"unhomed: {rank_totals[i][2]}")
            print("  W_max =", solution["w_max"])
            print("  assignments =", assignments)

        else: # if self.problem.status == pulp.LpStatusOptimal
            print("# No solution found")

    def setup_milp(self):
        """ Solve a minimization of a mixed-integer linear program. """
        # instantiante LP minimization problem
        self.problem = pulp.LpProblem("CCM_MILP", pulp.LpMinimize)

        # chi: ranks <- tasks, self.I x self.K, binary variables in MILP
        self.chi = pulp.LpVariable.dicts("chi", ((i, k) for i in range(self.I) for k in range(self.K)), cat="Binary")

        # phi: ranks <- shared blocks, self.I x self.N, binary variables in MILP
        self.phi = pulp.LpVariable.dicts("phi", ((i, n) for i in range(self.I) for n in range(self.N)), cat="Binary")

        # psi: ranks <- communications self.I x self.I x self.M, binary variables in MILP
        self.psi = {}
        if self.config.is_fwmp:
            self.psi = pulp.LpVariable.dicts("psi", ((i, j, m)
                for i in range(self.I)
                for j in range(self.I)
                for m in range(self.M)
            ), cat="Binary")

        # w_max: continuous variable in MILP for work defined by CCM model
        self.w_max = pulp.LpVariable("w_max", lowBound=0, cat="Continuous")

        # Add the continuous variable to the problem
        self.problem += self.w_max
        n_constraints_equal, n_constraints_inequ = 0, 0

        # Add part of equation 14 constraining every task to a single rank
        start_time = time.perf_counter()
        n_constraints_added = 0
        for k in range(self.K):
            self.problem += sum(self.chi[i, k] for i in range(self.I)) == 1
            n_constraints_added += 1

        # Report on added constraints
        end_time = time.perf_counter()
        print(f"  added {n_constraints_added} consistency constraints (14) in {end_time - start_time:0.4f}s")
        n_constraints_equal += n_constraints_added

        # Add equations 18 and 19 for shared block constraints
        start_time = time.perf_counter()
        n_constraints_added = 0
        for i in range(self.I):
            for n in range(self.N):
                # Add equation 18
                for p in range(len(self.task_memory_block_mapping[n])):
                    self.problem += self.phi[i, n] >= self.chi[i, self.task_memory_block_mapping[n][p]]
                    n_constraints_added += 1

                # Add equation 19
                self.problem += self.phi[i, n] <= sum(
                    self.chi[i, self.task_memory_block_mapping[n][p]]
                    for p in range(len(self.task_memory_block_mapping[n])))
                n_constraints_added += 1

        # Report on added constraints
        end_time = time.perf_counter()
        print(f"  added {n_constraints_added} shared blocks constraints (18) & (19) in {end_time - start_time:0.4f}s")
        n_constraints_inequ += n_constraints_added

        n_constraints_notin = 0
        # Include rank memory constraints when requested
        if self.config.rank_memory_bound < math.inf:
            start_time = time.perf_counter()
            n_constraints_added = 0

            # Check whether all task memory overheads are nil
            all_task_M_overhead_zero = True
            for k in range(self.K):
                if self.task_M_overhead[k]:
                    all_task_M_overhead_zero = False

            # Add equation 20
            for i in range(self.I):
                if all_task_M_overhead_zero:
                    # Only one constraint when all task working bytes are nil
                    self.problem += (
                        sum(self.task_M_baseline[l] * self.chi[i, l] for l in range(self.K)) +
                        sum(self.memory_blocks[n] * self.phi[i, n] for n in range(self.N))
                    ) <= (self.rank_M_inf[i] - self.rank_M_baseline[i])
                    n_constraints_added += 1
                    n_constraints_notin += self.K - 1
                else:
                    # Generate all K constraints otherwise
                    for k in range(self.K):
                        self.problem += (
                            sum(self.task_M_baseline[l] * self.chi[i, l] for l in range(self.K)) +
                            self.task_M_overhead[k] * self.chi[i, k] +
                            sum(self.memory_blocks[n] * self.phi[i, n] for n in range(self.N))
                        ) <= (self.rank_M_inf[i] - self.rank_M_baseline[i])
                        n_constraints_added += 1

            # Report on added constraints
            end_time = time.perf_counter()
            print(f"  added {n_constraints_added} rank memory constraints (20) in {end_time - start_time:0.4f}s")
            n_constraints_inequ += n_constraints_added

        # Include node memory constraints when requested
        if self.config.node_memory_bound < math.inf:
            start_time = time.perf_counter()
            n_constraints_added = 0

            # Check whether all task memory overheads are nil
            all_task_M_overhead_zero = True
            for k in range(self.K):
                if self.task_M_overhead[k]:
                    all_task_M_overhead_zero = False

            # Add equation 21
            for h in range(self.I // self.Q):
                if all_task_M_overhead_zero:
                    # Only one constraint when all task working bytes are nil
                    self.problem += (sum(
                        sum(self.task_M_baseline[l] * self.chi[i, l] for l in range(self.K)) +
                        sum(self.memory_blocks[n] * self.phi[i, n] for n in range(self.N))
                        for i in range(h * self.Q, (h + 1) * self.Q))
                    ) <= (self.node_M_inf[h] - sum(
                        self.rank_M_baseline[i] for i in range(h * self.Q, (h + 1) * self.Q)))
                    n_constraints_added += 1
                    n_constraints_notin += self.K - 1
                else:
                    # Generate all K constraints otherwise
                    for k in range(self.K):
                        self.problem += (sum(
                            sum(self.task_M_baseline[l] * self.chi[i, l] for l in range(self.K)) +
                            self.task_M_overhead[k] * self.chi[i, k] +
                            sum(self.memory_blocks[n] * self.phi[i, n] for n in range(self.N))
                            for i in range(h * self.Q, (h + 1) * self.Q))
                        ) <= (self.node_M_inf[h] - sum(
                            self.rank_M_baseline[i] for i in range((h - 1) * self.Q, h * self.Q)))
                        n_constraints_added += 1

            # Report on added constraints
            end_time = time.perf_counter()
            print(f"  added {n_constraints_added} node memory constraints (21) in {end_time - start_time:0.4f}s")
            n_constraints_inequ += n_constraints_added

        # Add communication constraints 27 28 29 in full model case
        if self.config.is_fwmp:
            start_time = time.perf_counter()
            n_constraints_added = 0
            for i in range(self.I):
                for j in range(self.I):
                    # Iterate over task communications
                    for p,_ in enumerate(self.task_communications):
                        # Add equation 27
                        self.problem += self.psi[i, j, p] <= self.chi[i, self.task_communications[p][0]]
                        n_constraints_added += 1

                        # Add equation 28
                        self.problem += self.psi[i, j, p] <= self.chi[j, self.task_communications[p][1]]
                        n_constraints_added += 1

                        # Add equation 29
                        self.problem += (
                              self.psi[i, j, p] >= self.chi[i, self.task_communications[p][0]]
                            + self.chi[j, self.task_communications[p][1]] - 1)
                        n_constraints_added += 1

            # Report on added constraints
            end_time = time.perf_counter()
            print(f"  added {n_constraints_added} communications constraints (27) -- (29) in {end_time - start_time:0.4f}s")
            n_constraints_inequ += n_constraints_added

        # Add continuous constraints
        start_time = time.perf_counter()
        n_constraints_added = 0
        if self.config.is_comcp:
            # Add equation 22
            for i in range(self.I):
                self.problem += sum(self.task_loads[k] * self.chi[i, k] for k in range(self.K)) <= self.w_max
                n_constraints_added += 1
        elif self.config.is_fwmp:
            # Add equation 32
            for i in range(self.I):
                # For rank i, build a list of all the remote shared blocks for the forth term of equation 30
                remote_blocks = [n for n in range(self.N) if self.memory_block_home[n] != i]

                # For rank i, build a list of all the other machines (all but i) for the second term of equation 30
                other_machines = [j for j in range(self.I) if j != i]

                # Compute unchanging terms in equation 30
                alpha_term = self.config.alpha * sum(
                    self.task_loads[k] * self.chi[i, k] for k in range(self.K))
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
                n_constraints_added += 1

                # Add equation 30 for second transposition of beta term (σ(i,j) = {j,i})
                self.problem += alpha_term + gamma_term + delta_term + self.config.beta * sum(
                    self.psi[j, i, p] * self.task_communications[p][2]
                    for j in other_machines for p in range(len(self.task_communications))) <= self.w_max
                n_constraints_added += 1
        else:
            # No model selected
            print("*** ERROR: no model was selected")
            sys.exit(1)

        # Report on added constraints
        end_time = time.perf_counter()
        print(f"  added {n_constraints_added} continuous constraints ({'2' if self.config.is_comcp else '3'}2) in {end_time - start_time:0.4f}s")
        n_constraints_inequ += n_constraints_added

        # Add cluster-preserving constraints 16 when requested
        if self.config.preserve_clusters:
            start_time = time.perf_counter()
            n_constraints_added = 0
            for n in range(self.N):
                self.problem += sum(self.phi[i, n] for i in range(self.I)) == 1
                n_constraints_added += 1
            end_time = time.perf_counter()
            print(f"  added {n_constraints_added} cluster-preserving constraints (16) in {end_time - start_time:0.4f}s")
            n_constraints_equal += n_constraints_added

        # Check that correct number of equality constraints were added
        n_constraints_theory = self.K
        if self.config.preserve_clusters:
            n_constraints_theory += self.N
        if n_constraints_equal == n_constraints_theory:
            print(f"  {n_constraints_equal} equality constraints correctly added")
        else:
            print(f"*** ERROR: incorrect number of equality constrainsts added: {n_constraints_equal} <> {n_constraints_theory}")
            sys.exit(1)

        # Report on computed vs theoretical maximum number of inequality constraints were added
        n_constraints_theory = self.I * ((self.K + 1) * self.N + 1)
        if self.config.is_fwmp:
            n_constraints_theory += self.I * (3 * self.I * self.M + 1)
        if self.config.rank_memory_bound < math.inf:
            n_constraints_theory += self.I * self.K
        if self.config.node_memory_bound < math.inf:
            n_constraints_theory += self.I * self.K // self.Q
        print(f"  {n_constraints_inequ} inequality constraints added out of a theoretical maximum of {n_constraints_theory}")

    def write_lp_to_file(self, file_name : str):
        """Generate the problem file .pl"""
        self.problem.writeLP(file_name)

    def write_mps_to_file(self, file_name : str):
        """Generate the problem file .mps"""
        self.problem.writeMPS(file_name)

    @staticmethod
    def permute(permutation_file: str, data_files: list, file_prefix: str):
        """Apply permutation"""
        new_contents = {}
        new_task_map = {}
        new_communication_map = {}
        task_id = 0

        # Read permutation data
        permutation = {}
        with open(permutation_file, 'r', encoding="utf-8") as f:
            permutation = json.load(f)

        print(f"\nProcess permutation={permutation}\n")

        # Permute sorted data
        def sort_func(filepath):
            """Sort using the int into the file name"""
            return len(filepath.split('.')) > 1 and filepath.split('.')[1] or -1
        data_files.sort(key=sort_func)

        # Sort input data file
        for data_file in data_files:

            # Get content file
            data_json = None
            with open(data_file, 'r', encoding="utf-8") as f:
                data_json = json.load(f)

            # Get rank from filename (/some/path/data.{rank}.json})
            rank: int = -1
            if len(data_file.split('.')) > 1:
                split_data_filename =  data_file.split('.')
                rank = int(split_data_filename[len(split_data_filename) - 2])

            # Get tasks
            tasks = data_json["phases"][0].get("tasks")
            if tasks is not None and len(tasks) > 0:
                for task in tasks:
                    if "user_defined" in task:
                        new_rank = permutation[task_id]

                        # Create dict
                        if new_task_map.get(new_rank) is None:
                            new_task_map[new_rank] = []

                        # Add task into the rank
                        new_task_map[new_rank].append(task)

                        # Increments counters
                        task_id += 1

            # Get communications
            communications = data_json["phases"][0].get("communications")
            if communications is not None and len(communications) > 0:
                for communication in communications:
                    # Get from task id
                    from_task = communication["from"]
                    from_task_id = from_task.get("id", from_task.get("seq_id"))

                    # Create dict
                    if new_communication_map.get(from_task_id) is None:
                        new_communication_map[from_task_id] = []

                    # Add communication into the rank
                    new_communication_map[from_task_id].append(communication)

            # New content
            new_contents[rank] = data_json

        # Create new JSON
        for data_file in data_files:
            # Get rank
            rank: int = -1
            if len(data_file.split('.')) > 1:
                split_data_filename =  data_file.split('.')
                rank = int(split_data_filename[len(split_data_filename) - 2])

            if rank in new_task_map:
                # Get phase index
                index_phase: int = 1

                # data json
                data_json = new_contents[rank]
                data_json["phases"].append({
                    "id": index_phase,
                    "tasks": []
                })

                # Apply permutations
                for task in new_task_map[rank]:
                    # Add task
                    data_json["phases"][index_phase]["tasks"].append(task)

                    # Get task id
                    entity = task.get("entity")
                    task_id = entity.get("id", entity.get("seq_id"))

                    # add communications
                    if new_communication_map.get(task_id) is not None:
                        for commmunications in new_communication_map.get(task_id):
                            if data_json["phases"][index_phase].get("commmunications") is None:
                                data_json["phases"][index_phase]["commmunications"] = []

                            data_json["phases"][index_phase]["commmunications"].append(commmunications)
            else:
                print(f"NO PERMUTATION for data file: {data_file}, rank: {rank}\n")

            # Create output filename
            output_permuted_filename = file_prefix + data_file.split("/").pop()
            output_permuted_file = os.path.join(Generator.output_dir(), output_permuted_filename)

            print(f"Generate permuted file: {output_permuted_file}, rank: {rank}")

            # Create output file
            with open(output_permuted_file, 'w', encoding="utf-8") as f:
                json.dump(data_json, f)

    @staticmethod
    def parse_json(
            data_files: list,
            ranks_per_node: int = DP.ranks_per_node,
            rank_mem_bnd: float = DP.rank_memory_bound,
            node_mem_bnd: float = DP.node_memory_bound,
            verbose: bool =False) -> Data:
        """Parse json data files to python"""
        # Permute sorted data
        def sort_func(filepath):
            """Sort using the int into the file name"""
            return len(filepath.split('.')) > 1 and filepath.split('.')[1] or -1
        data_files.sort(key=sort_func)

        # Initialize and populate data object
        if verbose:
            print("\n# Data files:")
            for df in data_files:
                print(f"  {df}")
        data = Data(ranks_per_node, rank_mem_bnd, node_mem_bnd)
        data.parse_json(data_files, verbose)
        return data

    @staticmethod
    def solve_problem(problem, solver_name: str):
        """Solve the given problem"""

        # Get PuLP solver
        solver = pulp.getSolver(solver=solver_name, keepFiles=True)

        # Set solver
        problem.setSolver(solver)

        # Solve the problem
        problem.solve()

    @staticmethod
    def output_dir(output_dirname: str = "output") -> str:
        """Get and create output dir"""
        output_dir = os.path.join(os.path.dirname(__file__), "../..", output_dirname)
        if os.path.isdir(output_dir) is False:
            os.mkdir(output_dir)

        return output_dir

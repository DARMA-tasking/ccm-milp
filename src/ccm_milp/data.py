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

import json
import os
import re
from .configuration import Configuration

# Per-rank data files are named <name>.<rank>.json
_RANK_FILE = re.compile(r"(.+)\.(\d+)\.json")

def rank_of_file(path: str) -> int:
    """Rank encoded in a <name>.<rank>.json file name"""
    match = _RANK_FILE.fullmatch(os.path.basename(path))
    if match is None:
        raise ValueError(f"{path}: expected a file named <name>.<rank>.json")
    return int(match.group(2))

def find_rank_files(paths: list) -> list:
    """Expand directories into their rank files and return all files sorted by rank"""
    files = []
    for path in paths:
        if os.path.isdir(path):
            files += [os.path.join(path, n) for n in os.listdir(path) if _RANK_FILE.fullmatch(n)]
        elif os.path.isfile(path):
            files.append(path)
        else:
            raise FileNotFoundError(f"no such file or directory: {path}")
    if not files:
        raise ValueError(f"no <name>.<rank>.json files found in: {', '.join(paths) or '(no paths given)'}")
    ranks = sorted(rank_of_file(f) for f in files)
    stems = {_RANK_FILE.fullmatch(os.path.basename(f)).group(1) for f in files}
    if len(stems) > 1:
        raise ValueError(f"rank files from multiple datasets: {', '.join(sorted(stems))}")
    if ranks != list(range(len(files))):
        raise ValueError(f"expected ranks 0 to {len(files) - 1}, found {ranks}")
    return sorted(files, key=rank_of_file)

def load_rank_files(paths: list, config: Configuration, verbose: bool = False) -> "Data":
    """Load problem data from rank files or directories containing them"""
    files = find_rank_files(paths)
    if verbose:
        print("\n# Data files:")
        for f in files:
            print(f"  {f}")
    data = Data(config.ranks_per_node, config.rank_memory_bound, config.node_memory_bound)
    data.parse_json(files, verbose)
    return data

class Data:
    """Data file class used after parsing json data files"""

    def __init__(self, ranks_per_node: int, rank_mem_bnd: float, node_mem_bnd: float):
        self.ranks_per_node = ranks_per_node
        self.rank_mem_bnd = rank_mem_bnd
        self.node_mem_bnd = node_mem_bnd
        self.rank_mems = None
        self.rank_alphas = {}
        self.node_mems = None
        self.rank_working_bytes = None
        self.task_loads = None
        self.task_working_bytes = None
        self.task_footprint_bytes = None
        self.task_id = None
        self.memory_blocks = None
        self.memory_block_home = None
        self.task_memory_block_mapping = None
        self.task_communications = None

    def parse_json(self, data_files: list, verbose: bool):
        """Parse JSON data files"""
        tasks  = []
        tasks_working_bytes = []
        tasks_footprint_bytes  = []
        task_indices  = []
        task_rank_obj_id  = []
        task_shared_id_map = {}
        shared_id_map = {}
        shared_id_home  = {}
        task_id = 0
        comunications = []
        total_load = 0.0

        # Initialize ranks
        ranks = {i: 0 for i in range(len(data_files))}

        # Iterate over data files
        for data_file in data_files:
            rank = rank_of_file(data_file)

            # Get content file
            data_json = None
            with open(data_file, 'r', encoding="UTF-8") as f:
                # Load data
                data_json = json.load(f)
                # Manage relevant rank metedata when available
                if (md := data_json.get("metadata")):
                    if md.get("rank", rank) != rank:
                        raise ValueError(f"{data_file}: metadata rank {md['rank']} does not match file name")
                    if (ra := md.get("rank_alpha")) is not None:
                        self.rank_alphas[rank] = float(ra)

                # Manage tasks data
                if "tasks" in data_json["phases"][0]:
                    # For each tasks
                    for task in data_json["phases"][0]["tasks"]:
                        # Get data
                        time = task["time"]
                        entity = task.get("entity")
                        if entity is None:
                            raise ValueError(f"{data_file}: task {task} has no associated entity")
                        index = entity.get("index")
                        obj_id = entity.get("id", entity.get("seq_id"))
                        if obj_id is None:
                            raise ValueError(f"{data_file}: entity {entity} has neither an id nor a seq_id")

                        # Handle user defined data when it exists
                        if task.get("user_defined") is None:
                            continue
                        shared_id = task.get(
                            "user_defined").get("shared_id")
                        shared_bytes = task.get(
                            "user_defined").get("shared_bytes")
                        task_working_bytes = task.get(
                            "user_defined").get("task_working_bytes", 0)
                        task_footprint_bytes = task.get(
                            "user_defined").get("task_footprint_bytes", 0)
                        rank_working_bytes = task.get(
                            "user_defined").get("rank_working_bytes", 0)

                        # Set data
                        ranks[rank] = rank_working_bytes
                        shared_id_map[shared_id] = shared_bytes
                        shared_id_home[shared_id] = rank
                        tasks.append(time)
                        tasks_footprint_bytes.append(task_footprint_bytes)
                        tasks_working_bytes.append(task_working_bytes)
                        task_indices.append(index)
                        task_rank_obj_id.append(obj_id)
                        if shared_id not in task_shared_id_map:
                            task_shared_id_map[shared_id] = []
                        task_shared_id_map[shared_id].append(task_id)
                        total_load += time

                        # Manage counter
                        task_id += 1

                # Manage communication data
                if "communications" in data_json["phases"][0]:
                    for com in data_json["phases"][0]["communications"]:
                        from_task = com.get("from")
                        if from_task is None:
                            raise ValueError(f"{data_file}: no origin task for communication {com}")
                        to_task = com.get("to")
                        if to_task is None:
                            raise ValueError(f"{data_file}: no destination task for communication {com}")
                        comunications.append([
                            from_task.get("id", from_task.get("seq_id")),
                            to_task.get("id", to_task.get("seq_id")),
                            com["bytes"]])

        # All ranks have same memory bound
        self.rank_mems = [self.rank_mem_bnd] * len(ranks)
        self.rank_working_bytes =  list(ranks.values())

        # All nodes have same memory bound
        self.node_mems = [self.node_mem_bnd] * (len(ranks) // self.ranks_per_node)

        # Initialize tasks
        self.task_loads = tasks
        self.task_working_bytes = tasks_working_bytes
        self.task_footprint_bytes = tasks_footprint_bytes
        self.task_id = task_rank_obj_id
        self.task_id.sort()

        # Initialize shared memory blocks
        self.memory_blocks = list(shared_id_map.values())
        self.memory_blocks.sort()
        self.memory_block_home = list(shared_id_home.values())
        self.memory_block_home.sort()
        self.task_memory_block_mapping = list(task_shared_id_map.values())
        self.task_memory_block_mapping.sort()

        # Initialize communications
        self.task_communications = comunications

        #  Print data object
        if verbose:
            print("\n# Data object:")
            print(f"  rank_mems:                 {self.rank_mems}")
            print(f"  node_mems:                 {self.node_mems}")
            print(f"  rank_working_bytes:        {self.rank_working_bytes}")
            print(f"  task_loads:                {self.task_loads}")
            print(f"  task_working_bytes:        {self.task_working_bytes}")
            print(f"  task_footprint_bytes:      {self.task_footprint_bytes}")
            print(f"  task_id:                   {self.task_id}")
            print(f"  memory_blocks:             {self.memory_blocks}")
            print(f"  memory_block_home:         {self.memory_block_home}")
            print(f"  task_memory_block_mapping: {self.task_memory_block_mapping}")
            print(f"  task_communications:       {self.task_communications}")

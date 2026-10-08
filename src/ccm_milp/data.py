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

def _entity_id(entity: dict, context: str):
    """Id of a task entity, which is either its id or its seq_id"""
    if entity is None:
        raise ValueError(f"{context} has no entity")
    obj_id = entity.get("id", entity.get("seq_id"))
    if obj_id is None:
        raise ValueError(f"{context} has neither an id nor a seq_id")
    return obj_id

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
        task_ids, task_loads, task_working_bytes, task_footprint_bytes = [], [], [], []
        block_bytes, block_home, block_tasks = {}, {}, {}
        entity_communications = []
        rank_working_bytes = [0] * len(data_files)

        for data_file in data_files:
            rank = rank_of_file(data_file)
            with open(data_file, 'r', encoding="UTF-8") as f:
                data_json = json.load(f)

            # Manage relevant rank metadata when available
            if (md := data_json.get("metadata")):
                if md.get("rank", rank) != rank:
                    raise ValueError(f"{data_file}: metadata rank {md['rank']} does not match file name")
                if (ra := md.get("rank_alpha")) is not None:
                    self.rank_alphas[rank] = float(ra)

            phase = data_json["phases"][0]
            for task in phase.get("tasks", []):
                task_index = len(task_ids)
                task_ids.append(_entity_id(task.get("entity"), f"{data_file}: task {task}"))
                task_loads.append(task["time"])

                # Tasks without user-defined data use no memory and share no block
                user_defined = task.get("user_defined") or {}
                task_working_bytes.append(user_defined.get("task_working_bytes", 0))
                task_footprint_bytes.append(user_defined.get("task_footprint_bytes", 0))
                if "rank_working_bytes" in user_defined:
                    rank_working_bytes[rank] = user_defined["rank_working_bytes"]
                if (shared_id := user_defined.get("shared_id")) is not None:
                    if (shared_bytes := user_defined.get("shared_bytes")) is None:
                        raise ValueError(f"{data_file}: task {task} has a shared_id but no shared_bytes")
                    block_bytes[shared_id] = shared_bytes
                    block_home[shared_id] = rank
                    block_tasks.setdefault(shared_id, []).append(task_index)

            for com in phase.get("communications", []):
                entity_communications.append((
                    _entity_id(com.get("from"), f"{data_file}: origin of communication {com}"),
                    _entity_id(com.get("to"), f"{data_file}: destination of communication {com}"),
                    com["bytes"]))

        # Communications refer to tasks by entity id, while the problem refers to them by index
        task_index_of = {}
        for k, obj_id in enumerate(task_ids):
            if obj_id in task_index_of:
                raise ValueError(f"duplicate task id {obj_id}")
            task_index_of[obj_id] = k
        self.task_communications = []
        for from_id, to_id, size in entity_communications:
            for obj_id in (from_id, to_id):
                if obj_id not in task_index_of:
                    raise ValueError(f"communication references unknown task id {obj_id}")
            self.task_communications.append([task_index_of[from_id], task_index_of[to_id], size])

        # All ranks and all nodes have the same memory bound
        self.rank_mems = [self.rank_mem_bnd] * len(data_files)
        self.node_mems = [self.node_mem_bnd] * (len(data_files) // self.ranks_per_node)
        self.rank_working_bytes = rank_working_bytes

        self.task_id = task_ids
        self.task_loads = task_loads
        self.task_working_bytes = task_working_bytes
        self.task_footprint_bytes = task_footprint_bytes

        # Block arrays share one ordering by shared id so that they stay aligned
        shared_ids = sorted(block_bytes)
        self.memory_blocks = [block_bytes[b] for b in shared_ids]
        self.memory_block_home = [block_home[b] for b in shared_ids]
        self.task_memory_block_mapping = [block_tasks[b] for b in shared_ids]

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

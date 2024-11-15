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
import math

verbose = True

class Data:
    """Data file class used after parsing json data files"""

    def __init__(self, rank_mem_bnd: float, node_mem_bnd: float):
        self.rank_mem_bnd = rank_mem_bnd
        self.node_mem_bnd = node_mem_bnd
        self.rank_mems = None
        self.rank_working_bytes = None
        self.task_loads = None
        self.task_working_bytes = None
        self.task_footprint_bytes = None
        self.task_rank = None
        self.task_id = None
        self.memory_blocks = None
        self.memory_block_home = None
        self.task_memory_block_mapping = None
        self.task_communications = None

    def parse_json(self, data_files: list):
        """Parse JSON data files"""
        tasks  = []
        tasks_working_bytes = []
        tasks_footprint_bytes  = []
        task_indices  = []
        task_rank_obj_id  = []
        tasks_rank  = []
        task_shared_block_id_map = {}
        shared_block_id_map = {}
        shared_block_id_home  = {}
        task_id = 0
        ranks = {}
        comunications = []
        total_load = 0.0

        for data_file in data_files:
            # Get rank from filename (/some/path/data.{rank}.json})
            rank: int = -1
            if len(data_file.split('.')) > 1:
                split_data_filename =  data_file.split('.')
                rank = int(split_data_filename[len(split_data_filename) - 2])

            # Get content file
            data_json = None
            with open(data_file, 'r', encoding="UTF-8") as f:
                # Load data
                data_json = json.load(f)

                # Manage Tasks data
                if "tasks" in data_json["phases"][0]:
                    # Even if rank as no task we set mems to 0
                    if len(data_json["phases"][0]["tasks"]) < 1:
                        ranks[rank] = 0

                    # For each tasks
                    for task in data_json["phases"][0]["tasks"]:
                        # Get data
                        time = task["time"]
                        index = task.get("entity").get("index")
                        obj_id = task.get("entity").get("id")
                        if task.get("user_defined") is None:
                            continue
                        shared_block_id = task.get("user_defined").get("shared_block_id")
                        shared_bytes = task.get("user_defined").get("shared_bytes")
                        task_working_bytes = task.get("user_defined").get("task_working_bytes", 0)
                        task_footprint_bytes = task.get("user_defined").get("task_footprint_bytes", 0)
                        rank_working_bytes = task.get("user_defined").get("rank_working_bytes", 0)

                        # Set data
                        ranks[rank] = rank_working_bytes
                        shared_block_id_map[shared_block_id] = shared_bytes
                        shared_block_id_home[shared_block_id] = rank
                        tasks.append(time)
                        tasks_footprint_bytes.append(task_footprint_bytes)
                        tasks_working_bytes.append(task_working_bytes)
                        task_indices.append(index)
                        tasks_rank.append( rank)
                        task_rank_obj_id.append(obj_id)
                        if shared_block_id not in task_shared_block_id_map:
                            task_shared_block_id_map[shared_block_id] = []
                        task_shared_block_id_map[shared_block_id].append(task_id)
                        total_load += time

                        # Manage counter
                        task_id += 1

                # Manage communication data
                if "communications" in data_json["phases"][0]:
                    for com in data_json["phases"][0]["communications"]:
                        comunications.append([
                            com["from"]["id"],
                            com["to"]["id"],
                            com["bytes"]
                        ])

        # All ranks have same memory bound
        self.rank_mems = [self.rank_mem_bnd] * len(ranks)
        self.rank_working_bytes =  list(ranks.values())

        # Initialize tasks
        self.task_loads = tasks
        self.task_working_bytes = tasks_working_bytes
        self.task_footprint_bytes = tasks_footprint_bytes
        self.task_rank = tasks_rank
        self.task_id = task_rank_obj_id
        self.task_id.sort()

        # Initialize shared memory blocks
        self.memory_blocks = list(shared_block_id_map.values())
        self.memory_blocks.sort()
        self.memory_block_home = list(shared_block_id_home.values())
        self.memory_block_home.sort()
        self.task_memory_block_mapping = list(task_shared_block_id_map.values())
        self.task_memory_block_mapping.sort()

        # Initialize communications
        self.task_communications = comunications

        #  Print data object
        if verbose:
            print("\n# Data object:")
            print(f"  rank_mems:                 {self.rank_mems}")
            print(f"  rank_working_bytes:        {self.rank_working_bytes}")
            print(f"  task_loads:                {self.task_loads}")
            print(f"  task_working_bytes:        {self.task_working_bytes}")
            print(f"  task_footprint_bytes:      {self.task_footprint_bytes}")
            print(f"  task_rank:                 {self.task_rank}")
            print(f"  task_id:                   {self.task_id}")
            print(f"  memory_blocks:             {self.memory_blocks}")
            print(f"  memory_block_home:         {self.memory_block_home}")
            print(f"  task_memory_block_mapping: {self.task_memory_block_mapping}")
            print(f"  task_communications:       {self.task_communications}")

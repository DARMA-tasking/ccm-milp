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
from .data import find_rank_files, rank_of_file

def write_permutation(assignments: list, path: str):
    """Write the rank assigned to each task as a JSON list"""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, 'w', encoding="utf-8") as f:
        json.dump(assignments, f)

def permute(permutation_file: str, data_files: list, output_dir: str) -> list:
    """Apply a permutation to rank files, writing permuted copies to output_dir and returning their paths"""
    new_contents = {}
    new_task_map = {}
    new_communication_map = {}
    task_id = 0

    # Read permutation data
    permutation = {}
    with open(permutation_file, 'r', encoding="utf-8") as f:
        permutation = json.load(f)

    print(f"\nProcess permutation={permutation}\n")

    # Iterate over data files in rank order, matching the task order used to build the problem
    data_files = find_rank_files(data_files)
    os.makedirs(output_dir, exist_ok=True)
    output_files = []
    for data_file in data_files:

        # Get content file
        data_json = None
        with open(data_file, 'r', encoding="utf-8") as f:
            data_json = json.load(f)

        rank = rank_of_file(data_file)

        # Get tasks
        tasks = data_json["phases"][0].get("tasks")
        if tasks is not None and len(tasks) > 0:
            for task in tasks:
                if task_id >= len(permutation):
                    raise ValueError(f"permutation has {len(permutation)} entries but the data has more tasks")
                new_task_map.setdefault(permutation[task_id], []).append(task)
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

    if task_id != len(permutation):
        raise ValueError(f"permutation has {len(permutation)} entries but the data has {task_id} tasks")

    # Create new JSON
    for data_file in data_files:
        rank = rank_of_file(data_file)

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
                    data_json["phases"][index_phase].setdefault("communications", []).extend(
                        new_communication_map[task_id])
        else:
            print(f"NO PERMUTATION for data file: {data_file}, rank: {rank}\n")

        # Create output file
        output_permuted_file = os.path.join(output_dir, os.path.basename(data_file))
        print(f"Generate permuted file: {output_permuted_file}, rank: {rank}")
        with open(output_permuted_file, 'w', encoding="utf-8") as f:
            json.dump(data_json, f)
        output_files.append(output_permuted_file)

    return output_files

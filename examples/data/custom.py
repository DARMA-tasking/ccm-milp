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
import math

n_ranks = 9
n_per_node = 1
n_tasks = 72
n_blocks = 17

class Custom:
    def __init__(self):
        self.node_mems = [math.inf] * (n_ranks // n_per_node)
        self.rank_mems = [2.0] * n_ranks
        self.rank_working_bytes = [0.0] * n_ranks
        self.task_id = list(range(n_tasks))
        self.task_loads = [1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.5, 1.5, 1.5, 1.5, 0.0, 6.0, 6.0, 6.0, 1.5, 3.0, 3.0, 3.0, 1.0, 1.0, 3.0, 3.0, 1.0, 1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0, 1.0, 3.0, 3.0, 10.0, 10.0, 1.0, 1.0, 3.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
        self.task_working_bytes = [0.0] * n_tasks
        self.task_footprint_bytes = [0.0] * n_tasks
        self.memory_blocks = [1.0] * n_blocks
        self.memory_block_home = []
        self.task_memory_block_mapping = [[0, 1, 4, 5], [2, 3, 6, 7], [8, 9, 12, 13], [10, 11, 14, 15], [16, 17, 20, 21], [18, 19, 22, 23], [24, 25, 28, 29], [26, 27, 30, 31], [32, 33, 36, 37], [34, 35, 38, 39], [40, 41, 44, 45], [48, 49, 52, 53], [50, 51, 54, 55], [56, 57, 60, 61], [58, 59, 62, 63], [64, 65, 68, 69], [66, 67, 70, 71]]
        self.task_communications = []

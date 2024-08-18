#                           DARMA Toolkit v. 1.0.0
# 
# Copyright 2019 National Technology & Engineering Solutions of Sandia, LLC
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

class SyntheticBlocks:
    def __init__(self):
        self.rank_mems = [20] * 4
        self.rank_working_bytes = [0] * 4
        self.task_loads = [1.0, 0.5, 0.5, 0.5, 0.5, 2.0, 1.0, 0.5, 1.5]
        self.task_working_bytes = [0] * 9
        self.task_footprint_bytes = [0] * 9
        self.task_rank = [0, 0, 0, 0, 1, 1, 1, 1, 2]
        self.task_id = list(range(9))
        self.memory_blocks = [9] * 5
        self.memory_block_home = [0, 0, 1, 1, 2]
        self.task_memory_block_mapping = [[0, 1], [2, 3], [4, 5], [6, 7], [8]]
        self.task_communications = [[0, 5, 2.0], [1, 4, 1.0], [3, 2, 1.0], [3, 8, 0.5], [4, 1, 2.0], [5, 8, 2.0], [7, 6, 1.0], [8, 6, 1.5]]

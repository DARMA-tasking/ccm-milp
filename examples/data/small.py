#                           DARMA Toolkit v. 1.5.0
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

class SmallProblem:
    def __init__(self):
        self.rank_mems = [8000000000]*4
        self.rank_working_bytes = [980000000,980000000,980000000,980000000]
        self.task_loads = [10, 35, 10, 25, 10, 20, 10, 20, 15, 30, 5, 2.5, 5, 5, 5, 2.5, 15, 10, 5, 2.5, 10, 5, 2.5, 10, 15, 5, 10, 10, 5, 5, 20, 10]
        self.task_working_bytes = [110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000, 110000000]
        self.task_footprint_bytes = [1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024]
        self.task_rank = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3]
        self.task_id = [2883587, 2621443, 2359299, 2097155, 1835011, 524291, 262147, 786435, 1048579, 1310723, 1572867, 1310727, 1048583, 786439, 262151, 524295, 1835019, 524299, 262155, 786443, 1048587, 1310731, 1572875, 2359311, 2097167, 1835023, 524303, 262159, 786447, 1048591, 1310735, 1572879]
        self.memory_blocks = [1600000000,1600000000,1600000000,1600000000,1600000000,1600000000,1600000000,1600000000,1600000000,1600000000,1600000000,1600000000]
        self.memory_block_home = [0,1,2,3,0,1,2,3,0,1,2,3]
        self.task_memory_block_mapping = [[4,6,8],[12,14],[16,18,20],[25,27,29],[0,1,3,5,9],[11,15],[17,21],[24,26,30],[2,7,10],[13],[19,22],[23,28,31]]
        self.task_communications = [[1,4,1024*1024*1024],[5,6,2048*1024*1024],[8,2,4096*1024*1024],[8,1,8192*1024*1024],[2,16,8192*1024*1024]]

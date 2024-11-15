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
import sys
from dataclasses import dataclass

@dataclass
class Configuration:
    """Configuration object"""
    is_fmwp: bool
    is_comcp: bool
    alpha: float
    beta: float
    gamma: float
    delta: float
    rank_memory_bound: float
    node_memory_bound: float
    preserve_clusters: bool

    def __init__(
            self,
            is_fmwp: bool,
            alpha: float,
            beta: float,
            gamma: float,
            delta: float,
            rank_memory_bound: float,
            node_memory_bound: float,
            preserve_clusters: bool):
        print(f"\n# Initializing {'FMWP' if is_fmwp else 'COMCP'} configuration with:")
        print(f"  alpha = {alpha}")
        print(f"  beta = {beta}")
        print(f"  gamma = {gamma}")
        print(f"  delta = {delta}")
        if rank_memory_bound < math.inf:
            print(f"  rank memory bound = {rank_memory_bound}")
        if node_memory_bound < math.inf:
            print(f"  node memory bound = {node_memory_bound}")
        if preserve_clusters:
            print("  while preserving block clusters")
        self.is_fmwp = is_fmwp
        self.is_comcp = not is_fmwp
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.rank_memory_bound = rank_memory_bound
        self.node_memory_bound = node_memory_bound
        self.preserve_clusters = preserve_clusters

@dataclass
class DefaultParameters:
    """Default parameters"""
    alpha: float = 1.0
    beta: float = 0.0
    gamma: float = 0.0
    delta: float = 0.0
    rank_memory_bound: float = math.inf
    node_memory_bound: float = math.inf

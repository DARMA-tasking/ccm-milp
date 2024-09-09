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

class Config:
    """Config object"""
    def __init__(
        self,
        is_fmwp: bool,
        alpha: float,
        beta: float,
        gamma: float,
        delta: float,
        use_mem_ub:bool,
        preserve_clusters:bool
    ):
        print(f"\n# Initializing {'FMWP' if is_fmwp else 'COMCP'} configuration with:")
        print(f"  alpha = {alpha}")
        print(f"  beta = {beta}")
        print(f"  gamma = {gamma}")
        print(f"  delta = {delta}")
        print(f"  with{'' if use_mem_ub else 'out'} rank memory upper bound")
        self.is_fmwp = is_fmwp
        self.is_comcp = not is_fmwp
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.use_mem_ub = use_mem_ub
        self.preserve_clusters = preserve_clusters

class Examples:
    """Examples"""
    @staticmethod
    def list():
        """Examples list"""
        # Available CCM-MILP examples [filename, classname, RegexpTest[PULP_CBC_CMD & COIN_CMD, GLPK_CMD]]
        return [
            # example_id: 0
            ["small", "SmallProblem", [
                'Optimal - objective value 87.50000000', 
                'Objective:  OBJ = 87.5 (MINimum)'
            ]],

            # example_id: 1
            ["synthetic_blocks", "SyntheticBlocks", [
                'Optimal - objective value 2.00000000', 
                'Objective:  OBJ = 2 (MINimum)'
            ]],

            # example_id: 2
            ["ccm_example_no_sub_cluster", "CCMExampleNoSubCluster", [
                'Optimal - objective value 5.50000000', 
                'Objective:  OBJ = 5.5 (MINimum)'
            ]],

            # example_id: 3
            ["ccm_example_with_sub_cluster", "CCMExampleWithSubCluster", [
                'Optimal - objective value 5.50000000', 
                'Objective:  OBJ = 5.5 (MINimum)'
            ]],

            # example_id: 4
            ["illustration_1", "Illustration1", [
                'Optimal - objective value 20.00000000', 
                'Objective:  OBJ = 20 (MINimum)'
            ]],

            # example_id: 5
            ["illustration_2", "Illustration2", [
                'Optimal - objective value 20.00000000', 
                'Objective:  OBJ = 20 (MINimum)'
            ]]
        ]

class Parameters:
    """Parameters"""

    @staticmethod
    def defaults():
        """Default parameters list"""
        return {
            "alpha": 1.0,
            "beta": 0.0,
            "gamma": 0.0,
            "delta": 0.0
        }

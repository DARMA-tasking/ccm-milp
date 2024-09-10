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
        bounded_memory:bool,
        preserve_clusters:bool
    ):
        print(f"\n# Initializing {'FMWP' if is_fmwp else 'COMCP'} configuration with:")
        print(f"  alpha = {alpha}")
        print(f"  beta = {beta}")
        print(f"  gamma = {gamma}")
        print(f"  delta = {delta}")
        print(f"  with{'' if bounded_memory else 'out'} rank memory upper bound")
        if preserve_clusters:
            print("  while preserving block clusters")
        self.is_fmwp = is_fmwp
        self.is_comcp = not is_fmwp
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        self.bounded_memory = bounded_memory
        self.preserve_clusters = preserve_clusters

class ExampleConfig:
    """Examples Configs"""
    def __init__(
        self,
        filename: str = None,
        classname: str = None,
        test: bool = False,
        test_configs: any = None,
        test_regexp: dict = None
    ):
        self.filename = filename
        self.classname = classname
        self.test = test
        self.test_configs = test_configs
        self.test_regexp = test_regexp

class Examples:
    """Examples"""
    @staticmethod
    def list():
        """Examples list"""
        # Available CCM-MILP examples regexp_test [PULP_CBC_CMD & COIN_CMD, GLPK_CMD]
        return [
            ExampleConfig(
                filename = 'small',
                classname = 'SmallProblem'
            ),
            ExampleConfig(
                filename = 'synthetic_blocks',
                classname = 'SyntheticBlocks',
                test = True,
                test_configs = [
                    {
                        'file': '1-load-only.yaml',
                        'regexp': [
                            'Optimal - objective value 2.00000000',
                            'Objective:  OBJ = 2 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-load-only-and-cluster.yaml',
                        'regexp': [
                            'Optimal - objective value 2.50000000',
                            'Objective:  OBJ = 2.5 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-load-only-and-memory-bound.yaml',
                        'regexp': [
                            'Optimal - objective value 2.00000000',
                            'Objective:  OBJ = 2 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-fwmp-with-alpha.yaml',
                        'regexp': [
                            'Optimal - objective value 2.00000000',
                            'Objective:  OBJ = 2 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-fwmp-with-alpha-beta.yaml',
                        'regexp': [
                            'Optimal - objective value 4.00000000',
                            'Objective:  OBJ = 4 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-null-case.yaml',
                        'regexp': [
                            'Optimal - objective value 0.00000000',
                            'Objective:  OBJ = 0 (MINimum)'
                        ]
                    }
                ]
            ),
            ExampleConfig(
                filename = 'ccm_example_no_sub_cluster',
                classname = 'CCMExampleNoSubCluster'
            ),
            ExampleConfig(
                filename = 'ccm_example_with_sub_cluster',
                classname = 'CCMExampleWithSubCluster'
            ),
            ExampleConfig(
                filename = 'illustration_1',
                classname = 'Illustration1'
            ),
            ExampleConfig(
                filename = 'Illustration2',
                classname = '5'
            )
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

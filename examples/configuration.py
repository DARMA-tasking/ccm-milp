#                           DARMA Toolkit v. 1.0.0
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

import os

class ExampleConfig:
    """Examples Configs"""
    def __init__(
        self,
        filename: str = None,
        classname: str = None,
        json: list = [],
        test: bool = False,
        test_configs: any = None,
        test_regexp: dict = None
    ):
        self.filename = filename
        self.classname = classname
        self.json = json
        self.test = test
        self.test_configs = test_configs
        self.test_regexp = test_regexp

class Examples:
    """Examples"""

    @staticmethod
    def list():
        """Examples list"""
        # Get src dir
        data_dir = os.path.join(os.path.dirname(__file__), "..", "data")

        # Available CCM-MILP examples regexp_test [PULP_CBC_CMD & COIN_CMD, GLPK_CMD]
        return [
            ExampleConfig(
                filename = 'small',
                classname = 'SmallProblem'
            ),
            ExampleConfig(
                filename = 'synthetic_blocks',
                classname = 'SyntheticBlocks',
                json = [
                    os.path.join(data_dir, "synthetic_blocks", "data.0.json"),
                    os.path.join(data_dir, "synthetic_blocks", "data.1.json"),
                    os.path.join(data_dir, "synthetic_blocks", "data.2.json"),
                    os.path.join(data_dir, "synthetic_blocks", "data.3.json")
                ],
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
                    },
                    {
                        'file': '1-off-node-communication-only.yaml',
                        'regexp': [
                            'Optimal - objective value 0.00000000',
                            'Objective:  OBJ = 0 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-load-no-memory-homing-01.yaml',
                        'regexp': [
                            'Optimal - objective value 2.50000000',
                            'Objective:  OBJ = 2.5 (MINimum)'
                        ]
                    },
                    {
                        'file': '1-load-no-memory-homing-03.yaml',
                        'regexp': [
                            'Optimal - objective value 4.00000000',
                            'Objective:  OBJ = 4 (MINimum)'
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
                filename = 'illustration_2',
                classname = 'Illustration2'
            )
        ]

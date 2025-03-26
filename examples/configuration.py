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

import os
import fnmatch

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

    # Set data directories
    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")

    @staticmethod
    def list():
        """Examples list"""

        # Available CCM-MILP examples regexp_test [PULP_CBC_CMD & COIN_CMD, GLPK_CMD]
        return [
            ExampleConfig(
                filename = "small",
                classname = "SmallProblem"
            ),
            ExampleConfig(
                filename = "synthetic_blocks",
                classname = "SyntheticBlocks",
                json = Examples.list_data_files("synthetic_blocks"),
                test = True,
                test_configs = [
                    {
                        "file": "1-load-only.yaml",
                        "regexp": [
                            "Optimal - objective value 2.00000000",
                            "Objective:  OBJ = 2 (MINimum)"
                        ]
                    },
                    {
                        "file": "1-load-only-and-cluster.yaml",
                        "regexp": [
                            "Optimal - objective value 2.50000000",
                            "Objective:  OBJ = 2.5 (MINimum)"
                        ]
                    },
                    {
                        "file": "1-load-only-and-memory-bound.yaml",
                        "regexp": [
                            "Optimal - objective value 2.00000000",
                            "Objective:  OBJ = 2 (MINimum)"
                        ]
                    },
                    {
                        "file": "1-fwmp-with-alpha.yaml",
                        "regexp": [
                            "Optimal - objective value 2.00000000",
                            "Objective:  OBJ = 2 (MINimum)"
                        ]
                    },
                    {
                        "file": "1-fwmp-with-alpha-beta.yaml",
                        "regexp": [
                            "Optimal - objective value 4.00000000",
                            "Objective:  OBJ = 4 (MINimum)"
                        ]
                    },
                    {
                        "file": "1-load-no-memory-homing-01.yaml",
                        "regexp": [
                            "Optimal - objective value 2.50000000",
                            "Objective:  OBJ = 2.5 (MINimum)"
                        ]
                    },
                    {
                        "file": "1-load-no-memory-homing-03.yaml",
                        "regexp": [
                            "Optimal - objective value 4.00000000",
                            "Objective:  OBJ = 4 (MINimum)"
                        ]
                    }
                ]
            ),
            ExampleConfig(
                filename = "synthetic_blocks_alpha0",
                classname = "SyntheticBlocks",
                json = Examples.list_data_files("synthetic_blocks_alpha0"),
                test = True,
                test_configs = [
                    {
                        "file": "2-null-case.yaml",
                        "regexp": [
                            "Optimal - objective value 0.00000000",
                            "Objective:  OBJ = 0 (MINimum)"
                        ]
                    },
                    {
                        "file": "2-off-node-communication-only.yaml",
                        "regexp": [
                            "Optimal - objective value 0.00000000",
                            "Objective:  OBJ = 0 (MINimum)"
                        ]
                    }
                ]
            ),
            ExampleConfig(
                filename = "synthetic_blocks_bad_rank0",
                classname = "SyntheticBlocks",
                json = Examples.list_data_files("synthetic_blocks_bad_rank0"),
                test = True,
                test_configs = [
                    {
                        "file": "3-load-only.yaml",
                        "regexp": [
                            "Optimal - objective value 3.00000000",
                            "Objective:  OBJ = 3 (MINimum)"
                        ]
                    }
                ]
            ),
            ExampleConfig(
                filename = "sand2025-00006-example",
                classname = "SAND2025_00006_Example"
            ),
            ExampleConfig(
                filename = "illustration_1",
                classname = "Illustration1"
            ),
            ExampleConfig(
                filename = "illustration_2",
                classname = "Illustration2"
            ),
            ExampleConfig(
                filename = "custom",
                classname = "Custom"
            )
        ]

    @staticmethod
    def list_data_files(example_name: str) -> list:
        """Fetch relevant data file paths from data directory"""
        # Determine file name pattern corresponding to example
        if example_name == "synthetic_blocks":
            data_file_pattern = "synthetic-dataset-blocks.*.json"
        elif example_name == "synthetic_blocks_alpha0":
            data_file_pattern = "synthetic-dataset-blocks-alpha0.*.json"
        elif example_name == "synthetic_blocks_bad_rank0":
            data_file_pattern = "synthetic-dataset-blocks-bad-rank0.*.json"
        else:
            print(f"*  WARNING: ignoring unkown test case: {file}")

        # Retrieve files corresponding to test case
        example_data_dir = os.path.join(Examples.data_dir, example_name)
        data_files = []
        for file in os.listdir(example_data_dir):
            if fnmatch.fnmatch(file, data_file_pattern):
                data_files.append(
                    os.path.join(example_data_dir, file))

        # Sort list of files
        data_files.sort()

        # Return the list of absolute path for each data files found
        return data_files

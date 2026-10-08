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

import glob
import os
import argparse
import importlib
import sys

# Import classes
from ccm_milp.configuration import Configuration, load_yaml
from ccm_milp.data import load_rank_files
from ccm_milp.generator import Generator
from ccm_milp.tools import Tools

# Add global path
sys.path.insert(0, os.path.dirname(os.path.join(os.path.dirname(__file__), "../..")))
Examples = Tools.import_class("examples.configuration", "Examples")

# Available CCM-MILP examples
avail_examples = Examples.list()

# Default CCM parameter values
DP = Configuration()

def run_interactive():
    """Run with interactive selection of example"""

    # Build example
    print("# Available examples:")
    for example_id, example_config in enumerate(avail_examples):
        print(f"  {example_id}) {example_config.filename}.{example_config.classname}")

    example_id = int(input("  example index: "))
    if example_id not in range(len(avail_examples)):
        raise ValueError(f"Unavailable example index: {example_id}")

    # Interactively get and return problem configuration
    print("\n# Model configuration:")
    if (is_fwmp := input("  FWMP [y/N]? ") == "y"):
        beta  = Tools.input_float("beta")
        gamma = Tools.input_float("gamma")
        delta = Tools.input_float("delta")
    else:
        beta, gamma, delta = DP.beta, DP.gamma, DP.delta
    config = Configuration(
        ranks_per_node=Tools.input_int("ranks per node", DP.ranks_per_node),
        is_fwmp=is_fwmp, beta=beta, gamma=gamma, delta=delta,
        rank_memory_bound=Tools.input_float("rank memory bound", DP.rank_memory_bound),
        node_memory_bound=Tools.input_float("node memory bound", DP.node_memory_bound),
        preserve_clusters=input("  preserve clusters [y/N]? ") == "y")
    verbose = input("  verbose output [y/N]? ") == "y"
    return avail_examples[example_id], '', verbose, config

def run_batch(file_name: str):
    """Run with a config file"""
    params = load_yaml(file_name)
    example_id = params.pop("example_id", None)
    ccm_example = avail_examples[int(example_id)] if example_id is not None else None
    file_stem = str(params.pop("file_stem", ''))
    verbose = bool(params.pop("verbose", False))
    if not ccm_example and file_stem == '':
        print ("*** ERROR: No CCM example was defined or file_stem provided")
        sys.exit(1)
    return ccm_example, file_stem, verbose, Configuration.from_dict(params)

def main():
    # Manage options
    parser = argparse.ArgumentParser(
        prog="CCM-MILP",
        description="Generate & solve a problem"
    )
    parser.add_argument("-c", "--config", help="The config.yaml file", default=None)
    parser.add_argument("-s", "--solver", help="The problem solver", default="PULP_CBC_CMD")

    # Get options
    args = parser.parse_args()
    file_name = args.config
    solver_name = args.solver

    # Retrieve parameters either interactively or in batch mode
    if file_name:
        ccm_example, file_stem, verbose, config = run_batch(file_name)
    else:
        ccm_example, file_stem, verbose, config = run_interactive()
    print(f"\n{config.summary()}")

    # Manage available examples with JSON
    data = None
    if ccm_example is not None and len(ccm_example.json) > 0:
        # Retrieve data from set of predefined examples
        data = load_rank_files(ccm_example.json, config, verbose)
    elif file_stem != '':
        # Retrieve data from specified file stem
        data = load_rank_files(glob.glob(f"{glob.escape(file_stem)}.*.json"), config, verbose)
    else:
        data = getattr(importlib.import_module("examples.data." + ccm_example.filename), ccm_example.classname)()

    # Generate linear program and solve it when requested
    ccm_milp_generator = Generator(config, data)
    if solver_name:
        # Generate and solve
        ccm_milp_generator.generate_problem_and_solve(solver_name)

        # Report solution to linear program when found
        permutation_file: str =  ccm_example.filename + "_" + "permutation.json"
        ccm_milp_generator.output_solution(permutation_file)

        if len(ccm_example.json) > 0:
            # Call permute function
            Generator.permute(
                permutation_file = os.path.join(Generator.output_dir(), permutation_file),
                data_files = ccm_example.json,
                file_prefix="permuted_"
            )

    else:
        # Generate only
        ccm_milp_generator.generate_problem()
        print("\n# No LP solve requested")

if __name__ == "__main__":
    main()

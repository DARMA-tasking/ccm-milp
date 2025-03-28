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
import re
import argparse
import importlib
import sys
import typing
import yaml

# Import classes
from ccm_milp.configuration import Configuration, DefaultParameters
from ccm_milp.generator import Generator
from ccm_milp.tools import Tools

# Add global path
sys.path.insert(0, os.path.dirname(os.path.join(os.path.dirname(__file__), "../..")))
Examples = Tools.import_class("examples.configuration", "Examples")

# Available CCM-MILP examples
avail_examples = Examples.list()

# Default CCM parameter values
DP: typing.Final = DefaultParameters()

def run_interactive():
    """Run with interactive selection of example"""

    # Build example
    print("# Available examples:")
    for example_id, example_config in enumerate(avail_examples):
        print(f"  {example_id}) {example_config.filename}.{example_config.classname}")

    example_id = int(input("  example index: "))
    if example_id not in range(len(avail_examples)):
        raise ValueError(f"Unvailable example index: {example_id}")

    # Interactively get and return problem configuration
    print("\n# Model configuration:")
    if (is_fwmp := input("  FWMP [y/N]? ") == "y"):
        beta  = Tools.input_float("beta")
        gamma = Tools.input_float("gamma")
        delta = Tools.input_float("delta")
    else:
        beta, gamma, delta = DP.beta, DP.gamma, DP.delta
    return [
        avail_examples[example_id],
        Tools.input_int("ranks per node", DP.ranks_per_node),
        is_fwmp, beta, gamma, delta,
        Tools.input_float("rank memory bound", DP.rank_memory_bound),
        Tools.input_float("node memory bound", DP.node_memory_bound),
        input("  preserve clusters [y/N]? ") == "y",
        input("  verbose output [y/N]? ") == "y"]

def run_batch(file_name: str):
    """Run with a config file"""

    # Try to read YAML configuration file
    parameters = {}
    try:
        with open(file_name, 'r', encoding="utf-8") as f:
            parameters = yaml.safe_load(f)
    except: # pylint: disable=bare-except
        print ("*** ERROR: Could not parse", file_name)
        sys.exit(1)

    # Parse parameters
    print("# Model configuration:")
    c_bool, c_int, c_float, ccm_example, file_stem = {}, {}, {}, None, ''
    for k, v in parameters.items():
        print(f"  {k}: {v}")
        if k in ("is_fwmp", "bounded_memory", "preserve_clusters", "verbose"):
            c_bool[k] = bool(v)
        elif k in ("ranks_per_node"):
            c_int[k] = int(v)
        elif k in ("beta", "gamma", "delta", "rank_memory_bound", "node_memory_bound"):
            c_float[k] = float(v)
        elif k == "example_id":
            ccm_example = avail_examples[int(v)]
        elif k in ("file_stem"):
            file_stem = str(v)
        else:
            print ("*** ERROR: Incorrect key:", k)
            sys.exit(1)

    # Retrieve and return problem configuration
    if not ccm_example and file_stem == '':
        print ("*** ERROR: No CCM example was defined or file_stem provided")
        sys.exit(1)

    return [
        ccm_example,
        c_int.get("ranks_per_node", DP.ranks_per_node),
        c_bool.get("is_fwmp", False),
        c_float.get("beta", DP.beta),
        c_float.get("gamma", DP.gamma),
        c_float.get("delta", DP.delta),
        c_float.get("rank_memory_bound", DP.rank_memory_bound),
        c_float.get("node_memory_bound", DP.node_memory_bound),
        c_bool.get("preserve_clusters", False),
        c_bool.get("verbose", False),
        file_stem
    ]

def get_num_ranks(file_prefix : str, file_suffix : str) -> int:
    data_dir = f"{os.sep}".join(file_prefix.split(os.sep)[:-1])
    pattern = re.compile(rf"^{file_prefix}.(\d+).{file_suffix}$")
    highest_rank = 0
    for name in os.listdir(data_dir):
        path = os.path.join(data_dir, name)
        match_result = pattern.search(path)
        if match_result:
            rank_id = int(match_result.group(1))
            highest_rank = max(highest_rank, rank_id)
    return highest_rank + 1

def get_rank_file_name(file_prefix : str, file_suffix : str, rank_id : int):
    return f"{file_prefix}.{rank_id}.{file_suffix}"

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
        ccm_example, rpn, fwmp, beta, gamma, delta, rank_mem_bnd, node_mem_bnd, pr_cl, verbose, file_stem = run_batch(file_name)
    else:
        ccm_example, rpn, fwmp, beta, gamma, delta, rank_mem_bnd, node_mem_bnd, pr_cl, verbose = run_interactive()
        file_stem = ''

    # Manage available examples with JSON
    data = None
    if ccm_example is not None and len(ccm_example.json) > 0:
        # Retrieve data from set of predefined examples
        data = Generator.parse_json(ccm_example.json, rpn, rank_mem_bnd, node_mem_bnd, verbose)
    elif file_stem != '':
        # Retrieve data from specified file stem
        n_ranks = get_num_ranks(file_stem, "json")
        print(f"# Number of detected ranks: {n_ranks}")
        files = [get_rank_file_name(file_stem, "json", rid) for rid in range(n_ranks)]
        data = Generator.parse_json(files, rpn, rank_mem_bnd, node_mem_bnd, verbose)
    else:
        data = getattr(importlib.import_module("examples.data." + ccm_example.filename), ccm_example.classname)()

    # Generate linear program and solve it when requested
    ccm_milp_generator = Generator(
        Configuration(rpn, fwmp, beta, gamma, delta, rank_mem_bnd, node_mem_bnd, pr_cl),
        data)
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

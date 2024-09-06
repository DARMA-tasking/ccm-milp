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

import sys
import getopt
import importlib
import yaml
import os

from modules.configuration import Config, Examples, Parameters
from modules.generator import CCM_MILP_Generator
from modules.tools import Tools

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../examples"))

# Available CCM-MILP examples
avail_examples = Examples.list()    
    
# Default CCM parameter values
default_parameters = Parameters.defaults()

def run_interactive():
    # Build example
    print("# Available examples:")
    for i, [file_name, class_name, reg] in enumerate(avail_examples):
        print(f"  {i}) {file_name}.{class_name}")
    example_id = int(input("  example index: "))
    if example_id not in range(len(avail_examples)):
        raise ValueError(f"unvailable example index: {example_id}")
    print()

    # Generate and solve linear problem
    print("# Model configuration:")
    ccm_example = avail_examples[example_id]
    ccm_problem = CCM_MILP_Generator(
        Config(
            (is_fwmp := (input("  FWMP [y/N]? ") == 'y')),
            Tools.inputFloat("alpha") if is_fwmp else default_parameters["alpha"],
            Tools.inputFloat("beta") if is_fwmp else default_parameters["beta"],
            Tools.inputFloat("gamma") if is_fwmp else default_parameters["gamma"],
            Tools.inputFloat("delta") if is_fwmp else default_parameters["delta"],

            input("  bounded memory [y/N]? ") == 'y',
            input("  preserve clusters [y/N]? ") == 'y'
        ),
        getattr(importlib.import_module(ccm_example[0]), ccm_example[1])())
    ccm_problem.launch()

def run_batch(file_name: str):
    # Try to read YAML configuration file
    parameters = {}
    try:
        with open(file_name, "r", encoding="utf-8") as f:
            parameters = yaml.safe_load(f)
    except: # pylint: disable=bare-except
        print ("*** Could not parse", file_name)
        sys.exit(1)

    # Parse parameters
    print("# Model configuration:")
    c_float, c_bool, ccm_example = {}, {}, None
    for k, v in parameters.items():
        print(f"  {k}: {v}")
        if k in ("alpha", "beta", "gamma", "delta"):
            c_float[k] = float(v)
        elif k in ("is_fwmp", "bounded_memory", "preserve_clusters"):
            c_bool[k] = bool(v)
        elif k == "example_id":
            ccm_example = avail_examples[int(v)]
        else:
            print ("*** Incorrect key:", k)
            sys.exit(1)

    # Generate and solve linear problem
    if not ccm_example:
        print ("*** No CCM example was defined")
        sys.exit(1)
    ccm_problem = CCM_MILP_Generator(
        Config(
            c_bool.get("is_fwmp", False),
            c_float.get("alpha", default_parameters["alpha"]),
            c_float.get("beta", default_parameters["beta"]),
            c_float.get("gamma", default_parameters["gamma"]),
            c_float.get("delta", default_parameters["delta"]),
            c_bool.get("bounded_memory", False),
            c_bool.get("preserve_clusters", False)),
        getattr(importlib.import_module(ccm_example[0]), ccm_example[1])())
    ccm_problem.launch()

def main(argv):
    # Parse possible command-line arguments
    try:
        opts, _ = getopt.getopt(argv,"c:")
    except getopt.GetoptError:
        print ("*** Usage:ccm_milp_problem.py [-c <YAML configuration file>]")
        sys.exit(1)

    # Default execution mode is interactive
    file_name = None
    for o, a in opts:
        if o == '-c':
            file_name = a

    # Run either interactively or in batch mode
    if file_name:
        run_batch(file_name)
    else:
        run_interactive()

if __name__ == "__main__":
    main(sys.argv[1:])
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

import argparse
import os
import sys
import pulp

from ccm_milp.generator import CcmMilpGenerator

def run_batch(file_name: str, solver_name: str):
    """Run with a config file"""

    # Init
    error = False

    # Check problem file
    if os.path.isfile(file_name) is False:
        error = True
        print("*** File not found: ", file_name)
    else:
        print("# FileName: ", file_name)

    # Check solver
    if solver_name not in pulp.listSolvers(onlyAvailable=True):
        error = True
        print("*** Solver not found: ", solver_name)
        print("*** Available LP solvers: ", pulp.listSolvers(onlyAvailable=True))
    else:
        print("# Solver: ", solver_name)

    # Has error
    if error is True:
        print("")
        sys.exit(1)

    # Load problem file
    _, problem = pulp.LpProblem.fromMPS(file_name)

    CcmMilpGenerator.solve_problem(problem, solver_name)

def main():
    """Main"""
    # Init
    default_mps = os.path.join(os.path.dirname(__file__), 'problem.mps')

    # Manage options
    parser = argparse.ArgumentParser(
        prog='CCM-MILP Solver',
        description='Generate & solve a problem'
    )
    parser.add_argument('-p', '--problem', help='The problem.mps file', default=default_mps)
    parser.add_argument('-s', '--solver', help="The problem solver", default='PULP_CBC_CMD')

    # Get options
    args = parser.parse_args()
    file_name = args.problem
    solver_name = args.solver

    # Retrieve parameters in batch mode
    if file_name:
        run_batch(file_name, solver_name)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()

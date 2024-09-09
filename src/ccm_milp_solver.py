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

import getopt
import os
import sys
import pulp

from CcmMilp.Generator import CcmMilpGenerator

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

def main(argv):
    """Main"""
    # Parse possible command-line arguments
    try:
        opts, _ = getopt.getopt(argv,"ms:")
    except getopt.GetoptError:
        print ("*** Usage: ccm_milp_solver.py [-m <Problem File>] [-s <Solver Name>]")
        sys.exit(1)

    # Default execution mode is interactive
    file_name = os.path.join(os.path.dirname(__file__), 'problem.mps')

    # Default solver value ['GLPK_CMD', 'PULP_CBC_CMD', 'COIN_CMD']
    solver_name = 'COIN_CMD'

    # Overided by options
    for o, a in opts:
        if o == '-m':
            file_name = a
        if o == '-s':
            solver_name = a

    # Run either interactively or in batch mode
    if file_name:
        run_batch(file_name, solver_name)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main(sys.argv[1:])

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

import argparse
import sys
from ccm_milp.app import load_run_request_from_file, merge_run_request, run_request

def main():
    parser = argparse.ArgumentParser(
        prog="CCM-MILP",
        description="Generate and optionally solve a problem without interactive prompts."
    )
    parser.add_argument("-c", "--config", help="YAML config file", default=None)
    parser.add_argument("-s", "--solver", help="Solver name", default="PULP_CBC_CMD")
    parser.add_argument("--generate-only", action="store_true", help="Generate LP and MPS files without solving")

    args = parser.parse_args()
    if not args.config:
        parser.exit(
            2,
            "*** ERROR: Interactive mode has been removed. Use --config here or run 'python src/ccm_milp_cli.py run --help'.\n",
        )

    try:
        request = load_run_request_from_file(args.config)
        request = merge_run_request(
            request,
            solver_name=None if args.generate_only else args.solver,
            generate_only=args.generate_only or None,
        )
        run_request(request)
    except Exception as exc:  # pylint: disable=broad-except
        parser.exit(1, f"*** ERROR: {exc}\n")

if __name__ == "__main__":
    main()

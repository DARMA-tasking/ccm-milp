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

from ccm_milp.app import permute_json_files

def main():
    """Main"""

    parser = argparse.ArgumentParser(
        prog="CCM-MILP permute",
        description="Permute data in function of assignements"
    )

    parser.add_argument("-f", "--permutation-file", help="The permutation file in JSON", default=None)
    parser.add_argument("json_files", nargs="*", help="Input JSON files")
    parser.add_argument("-i", "--input-json-files", help="Legacy space-separated input JSON files", default=None)
    parser.add_argument(
        "--input-json-file",
        action="append",
        dest="json_file_args",
        default=[],
        help="Input JSON file. Repeat for multiple files.",
    )
    parser.add_argument("-o", "--output-file-prefix", help="The output file prefix", default="permuted_")

    args = parser.parse_args()
    input_files = list(args.json_files)
    input_files.extend(args.json_file_args)
    if args.input_json_files:
        input_files.extend(args.input_json_files.split())

    if not args.permutation_file or not input_files:
        parser.exit(1, "*** ERROR: Missing permutation file or input JSON files.\n")

    try:
        permute_json_files(args.permutation_file, input_files, args.output_file_prefix)
    except Exception as exc:  # pylint: disable=broad-except
        parser.exit(1, f"*** ERROR: {exc}\n")

if __name__ == "__main__":
    main()

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

from ccm_milp.generator import CcmMilpGenerator

# Globales variables
file_separator: str = "#"

def run_batch(permutation_file: str, file_input_json_files: str, file_prefix: str):
    """Run permutation"""
    # Read input json data
    datafiles = file_input_json_files.split(file_separator)

    # Call permute function
    CcmMilpGenerator.permute(
        permutation_file = permutation_file,
        data_files = datafiles,
        file_prefix = file_prefix
    )

def main():
    """Main"""

    # Manage options
    parser = argparse.ArgumentParser(
        prog='CCM-MILP permute',
        description='Permute data in function of assignements'
    )

    parser.add_argument('-f', '--permutation-file', help='The permutation file in JSON', default=None)
    parser.add_argument('-i', '--input-json-files', help="The input data file in JSON", default=None)
    parser.add_argument('-o', '--output-file-prefix', help="The output file prefix", default="permuted_")

    # Get options
    args = parser.parse_args()
    permutation_file = args.permutation_file
    input_json_files = args.input_json_files
    output_file_prefix = args.output_file_prefix

    # Retrieve parameters in batch mode
    if permutation_file and output_file_prefix and input_json_files:
        run_batch(permutation_file, input_json_files, output_file_prefix)
    else:
        print(f"Missing aruments: (permutation-file = {permutation_file} / input-json-files = {input_json_files})")
        sys.exit(1)

if __name__ == "__main__":
    main()

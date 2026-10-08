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
import os
import sys
from dataclasses import fields
from .configuration import Configuration, load_yaml
from .data import load_rank_files
from .generator import Generator
from .permute import permute, write_permutation

# Flag names that differ from their Configuration field name
_FLAG_NAMES = {"is_fwmp": "--fwmp"}

def _add_parameter_flags(parser: argparse.ArgumentParser):
    """Add one flag per model parameter, defaulting to None so unset flags don't override the YAML file"""
    group = parser.add_argument_group("model parameters (override values from --config)")
    for field in fields(Configuration):
        flag = _FLAG_NAMES.get(field.name, f"--{field.name.replace('_', '-')}")
        if field.type is bool:
            group.add_argument(flag, dest=field.name, action=argparse.BooleanOptionalAction, default=None)
        else:
            group.add_argument(flag, dest=field.name, type=field.type, default=None,
                               help=f"default: {field.default}")

def _load_configuration(args: argparse.Namespace) -> Configuration:
    """Merge parameters from the YAML file with those given as flags"""
    params = load_yaml(args.config) if args.config else {}
    for field in fields(Configuration):
        if (value := getattr(args, field.name)) is not None:
            params[field.name] = value
    return Configuration.from_dict(params)

def _solve(args: argparse.Namespace) -> int:
    config = _load_configuration(args)
    print(config.summary())
    generator = Generator(config, load_rank_files(args.data, config, args.verbose))
    generator.generate_problem(args.output_dir)
    if args.generate_only:
        print(f"\n# Wrote problem files to {args.output_dir}")
        return 0

    solution = generator.solve(args.solver)
    generator.report_solution(solution)
    if not solution.is_optimal:
        return 1
    permutation_file = os.path.join(args.output_dir, "permutation.json")
    write_permutation(solution.assignments, permutation_file)
    permute(permutation_file, args.data, os.path.join(args.output_dir, "permuted"))
    return 0

def _permute(args: argparse.Namespace) -> int:
    permute(args.permutation, args.data, args.output_dir)
    return 0

def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ccm-milp", description="Generate and solve CCM load-balancing MILPs")
    commands = parser.add_subparsers(required=True, metavar="command")
    data_help = "directory of <name>.<rank>.json files, or the files themselves"

    solve = commands.add_parser("solve", help="generate and solve a problem from rank files")
    solve.add_argument("data", nargs="+", help=data_help)
    solve.add_argument("-c", "--config", help="YAML file of model parameters")
    solve.add_argument("-s", "--solver", default="PULP_CBC_CMD", help="PuLP solver name (default: %(default)s)")
    solve.add_argument("-o", "--output-dir", default="output", help="output directory (default: %(default)s)")
    solve.add_argument("--generate-only", action="store_true", help="write problem files without solving")
    solve.add_argument("-v", "--verbose", action="store_true", help="print parsed input data")
    _add_parameter_flags(solve)
    solve.set_defaults(func=_solve)

    perm = commands.add_parser("permute", help="apply an existing permutation to rank files")
    perm.add_argument("permutation", help="JSON list of the rank assigned to each task")
    perm.add_argument("data", nargs="+", help=data_help)
    perm.add_argument("-o", "--output-dir", default="output/permuted", help="output directory (default: %(default)s)")
    perm.set_defaults(func=_permute)
    return parser

def main(argv=None) -> int:
    """Entry point of the ccm-milp command"""
    args = _parser().parse_args(argv)
    try:
        return args.func(args)
    except (ValueError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    sys.exit(main())

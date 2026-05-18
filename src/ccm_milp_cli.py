import argparse
import os
import sys

from ccm_milp.app import (
    DP,
    RunRequest,
    list_examples,
    load_run_request_from_file,
    merge_run_request,
    parse_json_files,
    permute_json_files,
    run_request,
    solve_problem_file,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="CCM-MILP",
        description="Run CCM-MILP from an explicit non-interactive CLI.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="Generate and optionally solve a problem")
    run_parser.add_argument("-c", "--config", help="YAML config file")
    run_parser.add_argument("--example", dest="example_name", help="Built-in example filename or class name")
    run_parser.add_argument("--example-id", type=int, help="Built-in example index")
    run_parser.add_argument("--file-stem", help="Prefix for rank files like prefix.0.json")
    run_parser.add_argument(
        "--json-file",
        action="append",
        dest="json_files",
        default=[],
        help="Input JSON data file. Repeat for multiple files.",
    )
    run_parser.add_argument("--ranks-per-node", type=int, help="Ranks per node")
    run_parser.add_argument("--fwmp", action="store_true", default=None, help="Use the full work model")
    run_parser.add_argument("--beta", type=float, help="Communication weight beta")
    run_parser.add_argument("--gamma", type=float, help="Local communication weight gamma")
    run_parser.add_argument("--delta", type=float, help="Memory homing weight delta")
    run_parser.add_argument("--rank-memory-bound", type=float, help="Rank memory bound")
    run_parser.add_argument("--node-memory-bound", type=float, help="Node memory bound")
    run_parser.add_argument("--preserve-clusters", action="store_true", default=None)
    run_parser.add_argument("--verbose", action="store_true", default=None)
    run_parser.add_argument("--solver", dest="solver_name", help="Solver name", default=None)
    run_parser.add_argument("--generate-only", action="store_true", help="Generate LP and MPS files without solving")
    run_parser.add_argument("--output-stem", help="Stem used for output artifacts")
    run_parser.add_argument(
        "--permute-output-prefix",
        default=None,
        help="Prefix for permuted JSON output files after solving",
    )
    run_parser.set_defaults(handler=_handle_run)

    list_examples_parser = subparsers.add_parser("list-examples", help="List built-in examples")
    list_examples_parser.set_defaults(handler=_handle_list_examples)

    solve_parser = subparsers.add_parser("solve", help="Solve an existing MPS file")
    solve_parser.add_argument(
        "-p",
        "--problem",
        default=os.path.join(os.path.dirname(__file__), "problem.mps"),
        help="Problem MPS file",
    )
    solve_parser.add_argument("-s", "--solver", required=False, default="PULP_CBC_CMD", help="Solver name")
    solve_parser.set_defaults(handler=_handle_solve)

    parse_parser = subparsers.add_parser("parse-json", help="Parse measurement JSON files")
    parse_parser.add_argument("json_files", nargs="+", help="Input JSON files")
    parse_parser.add_argument("--ranks-per-node", type=int, default=DP.ranks_per_node)
    parse_parser.add_argument("--rank-memory-bound", type=float, default=DP.rank_memory_bound)
    parse_parser.add_argument("--node-memory-bound", type=float, default=DP.node_memory_bound)
    parse_parser.add_argument("--verbose", action="store_true")
    parse_parser.set_defaults(handler=_handle_parse_json)

    permute_parser = subparsers.add_parser("permute", help="Permute measurement JSON files")
    permute_parser.add_argument("--permutation-file", required=True, help="Permutation assignment JSON file")
    permute_parser.add_argument("--output-file-prefix", default="permuted_", help="Output file prefix")
    permute_parser.add_argument("json_files", nargs="+", help="Input JSON files")
    permute_parser.set_defaults(handler=_handle_permute)

    return parser


def _handle_run(args: argparse.Namespace) -> int:
    request = RunRequest()
    if args.config:
        request = load_run_request_from_file(args.config)

    request = merge_run_request(
        request,
        example_id=args.example_id,
        example_name=args.example_name,
        file_stem=args.file_stem,
        json_files=args.json_files,
        ranks_per_node=args.ranks_per_node,
        is_fwmp=args.fwmp,
        beta=args.beta,
        gamma=args.gamma,
        delta=args.delta,
        rank_memory_bound=args.rank_memory_bound,
        node_memory_bound=args.node_memory_bound,
        preserve_clusters=args.preserve_clusters,
        verbose=args.verbose,
        solver_name=args.solver_name,
        generate_only=args.generate_only or None,
        output_stem=args.output_stem,
        permute_output_prefix=args.permute_output_prefix,
    )
    run_request(request)
    return 0


def _handle_list_examples(_: argparse.Namespace) -> int:
    print("# Available examples:")
    for example in list_examples():
        json_note = " (json)" if example.json_files else ""
        print(f"  {example.example_id}) {example.filename}.{example.classname}{json_note}")
    return 0


def _handle_solve(args: argparse.Namespace) -> int:
    solve_problem_file(args.problem, args.solver)
    return 0


def _handle_parse_json(args: argparse.Namespace) -> int:
    parse_json_files(
        list(args.json_files),
        ranks_per_node=args.ranks_per_node,
        rank_mem_bnd=args.rank_memory_bound,
        node_mem_bnd=args.node_memory_bound,
        verbose=args.verbose,
    )
    return 0


def _handle_permute(args: argparse.Namespace) -> int:
    permute_json_files(args.permutation_file, list(args.json_files), args.output_file_prefix)
    return 0


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except Exception as exc:  # pylint: disable=broad-except
        parser.exit(1, f"*** ERROR: {exc}\n")


if __name__ == "__main__":
    sys.exit(main())
import importlib
import os
import re
import sys
import typing
from dataclasses import dataclass, field, replace

import pulp
import yaml

from ccm_milp.configuration import Configuration, DefaultParameters
from ccm_milp.generator import Generator
from ccm_milp.tools import Tools

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
Examples = Tools.import_class("examples.configuration", "Examples")

DP: typing.Final = DefaultParameters()


@dataclass(frozen=True)
class RunRequest:
    example_id: typing.Optional[int] = None
    example_name: typing.Optional[str] = None
    file_stem: str = ""
    json_files: list[str] = field(default_factory=list)
    ranks_per_node: int = DP.ranks_per_node
    is_fwmp: bool = False
    beta: float = DP.beta
    gamma: float = DP.gamma
    delta: float = DP.delta
    rank_memory_bound: float = DP.rank_memory_bound
    node_memory_bound: float = DP.node_memory_bound
    preserve_clusters: bool = False
    verbose: bool = False
    solver_name: typing.Optional[str] = "PULP_CBC_CMD"
    generate_only: bool = False
    output_stem: str = ""
    permute_output_prefix: str = "permuted_"


@dataclass(frozen=True)
class ExampleDescriptor:
    example_id: int
    filename: str
    classname: str
    json_files: list[str]


def list_examples() -> list[ExampleDescriptor]:
    """Return the available examples in a CLI-friendly shape."""
    return [
        ExampleDescriptor(
            example_id=index,
            filename=example.filename,
            classname=example.classname,
            json_files=list(example.json),
        )
        for index, example in enumerate(Examples.list())
    ]


def load_run_request_from_file(file_name: str) -> RunRequest:
    """Read a YAML config file into a validated run request."""
    try:
        with open(file_name, "r", encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream) or {}
    except OSError as exc:
        raise ValueError(f"Could not read config file: {file_name}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"Could not parse config file: {file_name}") from exc

    if not isinstance(parameters, dict):
        raise ValueError("Config file must contain a YAML mapping at the top level")

    allowed_keys = {
        "example",
        "example_id",
        "example_name",
        "file_stem",
        "json_files",
        "ranks_per_node",
        "is_fwmp",
        "beta",
        "gamma",
        "delta",
        "rank_memory_bound",
        "node_memory_bound",
        "preserve_clusters",
        "verbose",
        "solver",
        "solver_name",
        "generate_only",
        "output_stem",
        "permute_output_prefix",
    }
    unknown_keys = sorted(set(parameters) - allowed_keys)
    if unknown_keys:
        raise ValueError(f"Unsupported config keys: {', '.join(unknown_keys)}")

    json_files = parameters.get("json_files", [])
    if isinstance(json_files, str):
        json_files = [json_files]
    if not isinstance(json_files, list):
        raise ValueError("json_files must be a YAML list or string")

    example_name = parameters.get("example_name", parameters.get("example"))
    solver_name = parameters.get("solver_name", parameters.get("solver", "PULP_CBC_CMD"))

    return RunRequest(
        example_id=_coerce_optional_int(parameters.get("example_id")),
        example_name=str(example_name) if example_name is not None else None,
        file_stem=str(parameters.get("file_stem", "")),
        json_files=[str(path) for path in json_files],
        ranks_per_node=int(parameters.get("ranks_per_node", DP.ranks_per_node)),
        is_fwmp=bool(parameters.get("is_fwmp", False)),
        beta=float(parameters.get("beta", DP.beta)),
        gamma=float(parameters.get("gamma", DP.gamma)),
        delta=float(parameters.get("delta", DP.delta)),
        rank_memory_bound=float(parameters.get("rank_memory_bound", DP.rank_memory_bound)),
        node_memory_bound=float(parameters.get("node_memory_bound", DP.node_memory_bound)),
        preserve_clusters=bool(parameters.get("preserve_clusters", False)),
        verbose=bool(parameters.get("verbose", False)),
        solver_name=str(solver_name) if solver_name is not None else None,
        generate_only=bool(parameters.get("generate_only", False)),
        output_stem=str(parameters.get("output_stem", "")),
        permute_output_prefix=str(parameters.get("permute_output_prefix", "permuted_")),
    )


def merge_run_request(request: RunRequest, **overrides) -> RunRequest:
    """Return a copy of a run request with the provided non-None overrides."""
    changes = {}
    for key, value in overrides.items():
        if value is None:
            continue
        if key == "json_files" and not value:
            continue
        changes[key] = value
    return replace(request, **changes)


def run_request(request: RunRequest) -> None:
    """Generate and optionally solve a CCM-MILP problem."""
    example = _resolve_example(request)
    input_name, input_files, input_problem = _load_problem_input(request, example)
    generator = Generator(
        Configuration(
            request.ranks_per_node,
            request.is_fwmp,
            request.beta,
            request.gamma,
            request.delta,
            request.rank_memory_bound,
            request.node_memory_bound,
            request.preserve_clusters,
        ),
        input_problem,
    )

    if request.generate_only:
        generator.generate_problem()
        print("\n# No LP solve requested")
        return

    if not request.solver_name:
        raise ValueError("A solver name is required unless --generate-only is used")

    generator.generate_problem_and_solve(request.solver_name)

    permutation_file = f"{_get_output_stem(request, input_name)}_permutation.json"
    generator.output_solution(permutation_file)

    if input_files:
        Generator.permute(
            permutation_file=os.path.join(Generator.output_dir(), permutation_file),
            data_files=input_files,
            file_prefix=request.permute_output_prefix,
        )


def solve_problem_file(file_name: str, solver_name: str) -> None:
    """Solve an existing MPS file with a requested solver."""
    if not os.path.isfile(file_name):
        raise FileNotFoundError(f"File not found: {file_name}")

    available_solvers = pulp.listSolvers(onlyAvailable=True)
    if solver_name not in available_solvers:
        raise ValueError(
            f"Solver not found: {solver_name}. Available solvers: {', '.join(available_solvers)}"
        )

    print("# FileName:", file_name)
    print("# Solver:", solver_name)
    _, problem = pulp.LpProblem.fromMPS(file_name)
    Generator.solve_problem(problem, solver_name)


def parse_json_files(
    data_files: list[str],
    ranks_per_node: int = DP.ranks_per_node,
    rank_mem_bnd: float = DP.rank_memory_bound,
    node_mem_bnd: float = DP.node_memory_bound,
    verbose: bool = False,
):
    """Parse JSON measurement files into a CCM-MILP data object."""
    return Generator.parse_json(data_files, ranks_per_node, rank_mem_bnd, node_mem_bnd, verbose)


def permute_json_files(permutation_file: str, data_files: list[str], file_prefix: str = "permuted_") -> None:
    """Permute JSON measurement files using a permutation assignment file."""
    Generator.permute(permutation_file=permutation_file, data_files=data_files, file_prefix=file_prefix)


def get_rank_file_name(file_prefix: str, file_suffix: str, rank_id: int) -> str:
    return f"{file_prefix}.{rank_id}.{file_suffix}"


def get_num_ranks(file_prefix: str, file_suffix: str) -> int:
    data_dir = os.path.dirname(file_prefix)
    if not data_dir:
        data_dir = "."
    file_name_prefix = os.path.basename(file_prefix)
    pattern = re.compile(rf"^{re.escape(file_name_prefix)}\.(\d+)\.{re.escape(file_suffix)}$")
    highest_rank = -1
    for name in os.listdir(data_dir):
        match_result = pattern.search(name)
        if not match_result:
            continue
        highest_rank = max(highest_rank, int(match_result.group(1)))
    if highest_rank < 0:
        raise ValueError(f"No matching rank files found for prefix: {file_prefix}")
    return highest_rank + 1


def _coerce_optional_int(value) -> typing.Optional[int]:
    if value is None or value == "":
        return None
    return int(value)


def _resolve_example(request: RunRequest):
    examples = Examples.list()
    if request.example_id is not None:
        if request.example_id not in range(len(examples)):
            raise ValueError(f"Unavailable example index: {request.example_id}")
        return examples[request.example_id]

    if request.example_name:
        for example in examples:
            if example.filename == request.example_name or example.classname == request.example_name:
                return example
        raise ValueError(f"Unknown example: {request.example_name}")

    return None


def _load_problem_input(request: RunRequest, example):
    if request.json_files:
        data_files = list(request.json_files)
        data = Generator.parse_json(
            data_files,
            request.ranks_per_node,
            request.rank_memory_bound,
            request.node_memory_bound,
            request.verbose,
        )
        return _default_input_name_from_json_files(data_files), data_files, data

    if request.file_stem:
        rank_count = get_num_ranks(request.file_stem, "json")
        print(f"# Number of detected ranks: {rank_count}")
        data_files = [get_rank_file_name(request.file_stem, "json", rank_id) for rank_id in range(rank_count)]
        data = Generator.parse_json(
            data_files,
            request.ranks_per_node,
            request.rank_memory_bound,
            request.node_memory_bound,
            request.verbose,
        )
        return _default_input_name_from_file_stem(request.file_stem), data_files, data

    if example is not None and len(example.json) > 0:
        data_files = list(example.json)
        data = Generator.parse_json(
            data_files,
            request.ranks_per_node,
            request.rank_memory_bound,
            request.node_memory_bound,
            request.verbose,
        )
        return example.filename, data_files, data

    if example is not None:
        data = getattr(importlib.import_module("examples.data." + example.filename), example.classname)()
        return example.filename, [], data

    raise ValueError(
        "No input source selected. Provide --example, --example-id, --file-stem, --json-file, or --config."
    )


def _default_input_name_from_file_stem(file_stem: str) -> str:
    return os.path.basename(file_stem.rstrip(os.sep)) or "dataset"


def _default_input_name_from_json_files(data_files: list[str]) -> str:
    first_name = os.path.basename(data_files[0])
    return os.path.splitext(first_name)[0]


def _get_output_stem(request: RunRequest, input_name: str) -> str:
    stem = request.output_stem or input_name or "ccm_milp"
    return re.sub(r"[^A-Za-z0-9._-]+", "_", stem)
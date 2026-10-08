# CCM-MILP

Builds and solves the mixed-integer linear program (MILP) formulation of the CCM load-balancing problem: assigning tasks and their shared memory blocks to ranks so as to minimize the maximum work on any rank. Equation numbers in the code refer to [`doc/CCM-foundations.pdf`](doc/CCM-foundations.pdf).

## Installation

Requires Python 3.9 or later.

```shell
pip install -e .            # or: pip install -e ".[dev]" for pytest and pylint
```

This installs the `ccm-milp` command. Problems are solved through [PuLP](https://coin-or.github.io/pulp/), which ships with the CBC solver (`PULP_CBC_CMD`, the default). Any other solver PuLP can find is usable too, e.g. `GLPK_CMD` (`apt install glpk-utils`) or `GUROBI_CMD` (requires a Gurobi installation).

## Quickstart

```shell
# Solve a dataset with default parameters
ccm-milp solve data/synthetic_blocks

# Take parameters from a YAML file, overriding some with flags
ccm-milp solve data/synthetic_blocks -c tests/config/synthetic-blocks-fwmp-with-alpha-beta.yaml --beta 0.5

# Pass rank files explicitly, pick a solver and an output directory
ccm-milp solve data/synthetic_blocks/*.json --fwmp --ranks-per-node 2 -s GLPK_CMD -o results

# Only write the problem files, without solving
ccm-milp solve data/synthetic_blocks --generate-only

# Apply an existing permutation to rank files
ccm-milp permute output/permutation.json data/synthetic_blocks -o output/permuted
```

Run `ccm-milp solve --help` for all options.

### Outputs

Everything is written to the output directory (`./output` by default):

| File | Contents |
|---|---|
| `problem.lp`, `problem.mps` | The generated MILP, usable with any external solver |
| `permutation.json` | The rank assigned to each task, in input task order |
| `permuted/<name>.<rank>.json` | Copies of the input rank files with a second phase holding the optimal assignment |

A per-task and per-rank report of the solution is also printed. The command exits with status 1 when no optimal solution is found or the input is invalid.

## Input data

Problems are read from per-rank files in the VT [`LBDatafile`](https://darma-tasking.github.io/docs/html/node-lb-data.html#lb-data-file-format) JSON format. Only the first phase of each file is used, and the tasks in a rank's file are initially assigned to that rank.

```json
{
  "metadata": {"type": "LBDatafile", "rank": 0, "rank_alpha": 1.0},
  "phases": [{
    "id": 0,
    "tasks": [{
      "entity": {"type": "object", "seq_id": 0},
      "time": 1.0,
      "user_defined": {
        "shared_id": 0,
        "shared_bytes": 9.0,
        "task_working_bytes": 0,
        "task_footprint_bytes": 0,
        "rank_working_bytes": 0
      }
    }],
    "communications": [{
      "from": {"type": "object", "seq_id": 0},
      "to": {"type": "object", "seq_id": 5},
      "bytes": 2.0
    }]
  }]
}
```

| Field | Meaning | Default |
|---|---|---|
| `metadata.rank` | Rank of the file; must match the file name when present | file name |
| `metadata.rank_alpha` | Multiplier applied to the load of tasks on this rank, indicating relative slowness of the rank | 1.0 |
| `entity.id` or `entity.seq_id` | Unique task identifier, referenced by communications | required |
| `time` | Task load | required |
| `user_defined.shared_id` | Shared memory block used by the task; the block's home is the rank whose file holds its tasks | no block |
| `user_defined.shared_bytes` | Size of the shared memory block | required with `shared_id` |
| `user_defined.task_working_bytes` | Memory overhead of the task while it runs | 0 |
| `user_defined.task_footprint_bytes` | Baseline memory of the task | 0 |
| `user_defined.rank_working_bytes` | Baseline memory of the rank | 0 |
| `communications[].from`, `.to`, `.bytes` | Communication between two tasks, which may be listed in any rank file | none |

## Model parameters

Parameters can be set in a YAML file passed with `-c`, as flags, or both, with flags taking precedence. Unknown keys and invalid values are rejected.

| YAML key | Flag | Default | Meaning |
|---|---|---|---|
| `ranks_per_node` | `--ranks-per-node` | 1 | Ranks per node; must divide the number of ranks |
| `is_fwmp` | `--fwmp` / `--no-fwmp` | false | Use the full work model instead of the compute-only memory-constrained problem |
| `beta` | `--beta` | 0 | Work per byte of off-rank communication (full work model) |
| `gamma` | `--gamma` | 0 | Work per byte of on-rank communication (full work model) |
| `delta` | `--delta` | 0 | Work per byte of shared blocks used away from their home rank (full work model) |
| `rank_memory_bound` | `--rank-memory-bound` | `.inf` | Memory available on each rank |
| `node_memory_bound` | `--node-memory-bound` | `.inf` | Memory available on each node |
| `preserve_clusters` | `--preserve-clusters` / `--no-preserve-clusters` | false | Keep all tasks sharing a memory block on the same rank |

Example:

```yaml
is_fwmp: true
beta: 1.0e-8
rank_memory_bound: 8.0e9
preserve_clusters: true
```

## Included datasets

| Directory | Ranks | Tasks | Description |
|---|---|---|---|
| `data/synthetic_blocks` | 4 | 9 | Synthetic test case with 5 shared blocks |
| `data/synthetic_blocks_alpha0` | 4 | 9 | Same, with all rank alphas set to 0 |
| `data/synthetic_blocks_bad_rank0` | 4 | 9 | Same, with rank 0 a hundred times slower (`rank_alpha: 100`) |
| `data/small` | 4 | 32 | Larger problem with realistic memory sizes |
| `data/illustration_1`, `data/illustration_2` | 2 | 3 | Illustrations without and with communications |
| `data/sand2025_00006` | 2 | 3 | Example from report SAND2025-00006 |

## Development

```shell
pytest           # tests, using every solver PuLP finds available
tox              # tests on each supported Python version, plus pylint
```

The tests solve the synthetic datasets with the parameter files in `tests/config/` and check these optimal objective values:

| Parameter file | Dataset | Objective |
|---|---|---|
| `synthetic-blocks-load-only.yaml` | `synthetic_blocks` | 2 |
| `synthetic-blocks-load-only-and-cluster.yaml` | `synthetic_blocks` | 2.5 |
| `synthetic-blocks-load-only-and-memory-bound.yaml` | `synthetic_blocks` | 2 |
| `synthetic-blocks-fwmp-with-alpha.yaml` | `synthetic_blocks` | 2 |
| `synthetic-blocks-fwmp-with-alpha-beta.yaml` | `synthetic_blocks` | 4 |
| `synthetic-blocks-load-no-memory-homing-01.yaml` | `synthetic_blocks` | 2.5 |
| `synthetic-blocks-load-no-memory-homing-03.yaml` | `synthetic_blocks` | 4 |
| `synthetic-blocks-alpha0-null-case.yaml` | `synthetic_blocks_alpha0` | 0 |
| `synthetic-blocks-alpha0-off-node-communication-only.yaml` | `synthetic_blocks_alpha0` | 0 |
| `synthetic-blocks-bad-rank0-load-only.yaml` | `synthetic_blocks_bad_rank0` | 3 |

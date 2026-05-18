### Artifact: `assignments.f90`

## Description:
A Python program to build and solve the CCM-MILP problems, with a set of examples.

## Requirements:
* Python 3.x

## Execution
Run commands from the repository root.

### Unified CLI

List built-in examples:

```shell
python src/ccm_milp_cli.py list-examples
```

Generate and solve a built-in example:

```shell
python src/ccm_milp_cli.py run --example synthetic_blocks --solver PULP_CBC_CMD
```

Generate only from a YAML configuration file:

```shell
python src/ccm_milp_cli.py run --config tests/config/1-load-only.yaml --generate-only
```

Generate and solve from an external JSON file stem such as `prefix.0.json`, `prefix.1.json`, ...:

```shell
python src/ccm_milp_cli.py run --file-stem /abs/path/data/prefix --solver PULP_CBC_CMD
```

Generate and solve from explicit JSON files:

```shell
python src/ccm_milp_cli.py run \
    --json-file /abs/path/data.0.json \
    --json-file /abs/path/data.1.json \
    --solver PULP_CBC_CMD
```

The default solver is **PULP_CBC_CMD** unless `--generate-only` is used.

### Solve an Existing `.mps` File

Solve the generated `problem.mps` file:

```shell
python src/ccm_milp_cli.py solve
```

Solve a specific `.mps` file with a specific solver:

```shell
python src/ccm_milp_cli.py solve --problem /abs/path/problem.mps --solver GLPK_CMD
```

### Permute JSON Data Files

Permute JSON data and create output files:

```shell
python src/ccm_milp_cli.py permute \
    --permutation-file /abs/path/permutation.json \
    /abs/path/data.0.json /abs/path/data.1.json /abs/path/data.2.json
```

Specify a prefix for the output files:

```shell
python src/ccm_milp_cli.py permute \
    --permutation-file /abs/path/permutation.json \
    --output-file-prefix permuted_ \
    /abs/path/data.0.json /abs/path/data.1.json /abs/path/data.2.json
```

### Parse JSON

Parse JSON data and print the derived CCM-MILP data summary:

```shell
python src/ccm_milp_cli.py parse-json /abs/path/data.0.json /abs/path/data.1.json /abs/path/data.2.json
```

### Legacy Wrappers

The legacy scripts still exist, but they now use the same non-interactive API underneath:

```shell
python src/ccm_milp_run.py --config tests/config/1-load-only.yaml --generate-only
python src/ccm_milp_solver.py --problem /abs/path/problem.mps --solver PULP_CBC_CMD
python src/ccm_milp_parse_json.py /abs/path/data.0.json /abs/path/data.1.json
python src/ccm_milp_permute_json.py --permutation-file /abs/path/permutation.json /abs/path/data.0.json /abs/path/data.1.json
```

## What to Expect:
A text output in the terminal and a `.sol` file when it solve with the results of the optimization process, along with a `.lp` and `.mps` file containing the generated linear program.

For `SyntheticBlock` we also can retrieve in the `output` folder, after running `ccm_milp_full.py`, the JSON data files permuted and the permutation file generated and used for it.

## Expected Results:
By default the configuration is:

```YAML
beta:  0
gamma: 0
delta: 0
bounded_memory: false
preserve_clusters: false
```

The example tested is `SyntheticBlock` with these differents configurations:
* Load only
    * Configuration: `is_fwmp: false`
    * Optimal objective value `2.00000000`

* Load only and-cluster
    * Configuration: `is_fwmp: false, preserve_clusters: true`
    * Optimal objective value `2.50000000`

* Load only and-memory-bound
    * Configuration: `is_fwmp: false, bounded_memory: true`
    * Optimal objective value `2.00000000`

* FWMP with all rank alphas = 1.0
    * Configuration: `is_fwmp: true`
    * Optimal objective value `2.00000000`

* FWMP with all rank alphas = 1.0 and beta
    * Configuration: `is_fwmp: true, beta: 1`
    * Optimal objective value `4.00000000`

* Null case
    * Configuration: `is_fwmp: true, all alphas: 0, preserve_clusters: true`
    * Optimal objective value `0.00000000`

* Off node communication-only
    * Configuration: `is_fwmp: true, all alphas: 0, beta: 1`
    * Optimal objective value `0.00000000`

* Load no memory homing (delta: 0.1)
    * Configuration: `is_fwmp: true, delta: 0.1, preserve_clusters: true`
    * Optimal objective value `2.50000000`

* Load no memory homing (delta: 0.3)
    * Configuration: `is_fwmp: true, delta: 0.3, preserve_clusters: true`
    * Optimal objective value `4.00000000`

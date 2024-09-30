### Artifact: `assignments.f90`

## Description:
A Python program to build and solve the CCM-MILP problems, with a set of examples.

## Requirements:
* Python 3.x

## Execution
*From the folder `src`*

### Generate and solve a problem (and apply permutation on Synthetic Blocks example):
Either in interactive mode:

```shell
python ccm_milp_full.py
```

or in direct mode with a configuration file in YAML format:

```shell
python ccm_milp_full.py -c "<configuration file name>"
```

or with a specific solver installed on the machine:

```shell
python ccm_milp_full.py -s "<solver name: COIN_CMD, GLPK_CMD, PULP_CBC_CMD>"
```

By default the solver used is **PULP_CBC_CMD**

### Generate problem files (.pl and .mps):
Either in interactive mode:

```shell
python ccm_milp_problem.py
```

or in direct mode with a configuration file in YAML format:

```shell
python ccm_milp_problem.py -c "<configuration file name>"
```

### Solve a problem file (.mps):
Solve the generated problem.mps file generated before:

```shell
python ccm_milp_solver.py
```

Solve a specific .mps problem file:

```shell
python ccm_milp_solver.py -p "<problem file .mps>"
```

Solve with a specific solver installed on the machine:

```shell
python ccm_milp_solver.py -s "<solver name: COIN_CMD, GLPK_CMD, PULP_CBC_CMD>"
```


### Permutation for JSON data files using a permutation file:
*The seprator for `--input-json-files` files is a space*

Permute json data and create output files:

```shell
python ccm_milp_permute_json.py --permutation-file="/abs/path/permutation.json" --input-json-files="/abs/path/data.0.json /abs/path/data.1.json /abs/path/data.2.json ..."
```

Permute json data and create output files using a prefix for output files:

```shell
python ccm_milp_permute_json.py --output-file-prefix="permuted_" --permutation-file="/abs/path/permutation.json" --input-json-files="/abs/path/data.0.json /abs/path/data.1.json /abs/path/data.2.json ..."
```

### Parse JSON
*The seprator for files is a space*

Parse json data and output result into the terminal:

```shell
python ccm_milp_parse_json.py --input-json-files="/abs/path/data.0.json /abs/path/data.1.json /abs/path/data.2.json ..."
```

## What to Expect:
A text output in the terminal and a `.sol` file when it solve with the results of the optimization process, along with a `.lp` and `.mps` file containing the generated linear program.

For `SyntheticBlock` we also can retrieve in the `output` folder, after running `ccm_milp_full.py`, the JSON data files permuted and the permutation file generated and used for it.

## Expected Results:
By default the configuration is:

```YAML
alpha: 1
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

* FWMP with alpha
    * Configuration: `is_fwmp: true`
    * Optimal objective value `2.00000000`

* FWMP with alpha-beta
    * Configuration: `is_fwmp: true, beta: 1`
    * Optimal objective value `4.00000000`

* Null case
    * Configuration: `is_fwmp: true, alpha: 0, preserve_clusters: true`
    * Optimal objective value `0.00000000`

* Off node communication-only
    * Configuration: `is_fwmp: true, alpha: 0, beta: 1`
    * Optimal objective value `0.00000000`

* Load no memory homing (delta: 0.1)
    * Configuration: `is_fwmp: true, delta: 0.1, preserve_clusters: true`
    * Optimal objective value `2.50000000`

* Load no memory homing (delta: 0.3)
    * Configuration: `is_fwmp: true, delta: 0.3, preserve_clusters: true`
    * Optimal objective value `4.00000000`

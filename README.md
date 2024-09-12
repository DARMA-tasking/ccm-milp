### Artifact: `assignments.f90`

## Description:
A Python program to build and solve the CCM-MILP problems, with a set of examples.

## Requirements:
* Python 3.x

## Execution
from the folder `cd src/`

### Generate and solve a problem:

Either in interactive mode:

`python ccm_milp_full.py`

or in direct mode with a configuration file in YAML format:

`python ccm_milp_full.py -c <configuration file name>`

or with a specific solver installed on the machine:

`python ccm_milp_full.py -s <solver name: COIN_CMD, GLPK_CMD, PULP_CBC_CMD>`

By default the solver used is **PULP_CBC_CMD**

### Generate problem files (.pl and .mps):
Either in interactive mode:

`python ccm_milp_problem.py`

or in direct mode with a configuration file in YAML format:

`python ccm_milp_problem.py -c <configuration file name>`

### Solve a problem file (.mps):
Solve the generated problem.mps file generated before:

`python ccm_milp_solver.py`

Solve a specific .mps problem file:

`python ccm_milp_solver.py -p <problem file .mps>`

Solve with a specific solver installed on the machine:

`python ccm_milp_solver.py -s <solver name: COIN_CMD, GLPK_CMD, PULP_CBC_CMD>`

## What to Expect:
A text output in the terminal and a `.sol` file when it solve with the results of the optimization process, along with a `.lp` and `.mps` file containing the generated linear program.

## Expected Results:
By default the configuration is:

`alpha = 1, beta = 0, gamma = 0, delta= 0, bounded_memory = False, preserve_clusters = False`

The example tested is `SyntheticBlock` with these differents configurations:
* Load only
    * Configuration: `is_fwmp = False`
    * Optimal objective value `2.00000000`

* Load only and-cluster
    * Configuration: `is_fwmp = False, preserve_clusters = True`
    * Optimal objective value `2.50000000`

* Load only and-memory-bound
    * Configuration: `is_fwmp = False, bounded_memory: True`
    * Optimal objective value `2.00000000`

* FWMP with alpha
    * Configuration: `is_fwmp = True`
    * Optimal objective value `2.00000000`

* FWMP with alpha-beta
    * Configuration: `is_fwmp = True, beta = 1`
    * Optimal objective value `4.00000000`

* Null case
    * Configuration: `is_fwmp = True, alpha = 0, preserve_clusters = True`
    * Optimal objective value `0.00000000`

* Off node communication-only
    * Configuration: `is_fwmp = True, alpha = 0, beta = 1`
    * Optimal objective value `0.00000000`
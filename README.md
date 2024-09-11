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

# DARMA-tasking template repository

Template repository with base configuration.

Included workflows:
* [*check-pr-fixes-issue*](https://github.com/DARMA-tasking/check-pr-fixes-issue) - checking if PR description contains phrase "Fixes #issue", and if PR title, description and branch mention the same issue number
* [*find-unsigned-commits*](https://github.com/DARMA-tasking/find-unsigned-commits) - checking if there are any unsigned commits in PR
* [*find-trailing-whitespace*](https://github.com/DARMA-tasking/find-trailing-whitespace) - checking if there are any trailing whitespaces in files
* [*check-commit-format*](https://github.com/DARMA-tasking/check-commit-format) - checking if commit message is properly formatted - either starts with "*Merge ...*" or fullfils template: "*#issue_number: short commit description*"
* [*action-git-diff-check*](https://github.com/joel-coffman/action-git-diff-check) - checking if changes introduce conflict markers or whitespace errors

### Artifact: `assignments.f90`

#### Description:
A Fortran 90 program to verify the following MILP assignment constraints in the conference article: (16), (24), (25), (26), and (27) as realized for the examples of Figures 2 & 3.

#### Requirements:
* a Fortran 90-compatible compiler, e.g. [gfortran](https://gcc.gnu.org/wiki/GFortran);
* at least version 3.10 of [CMake](https://cmake.org/).

#### Build and Run on *nix Systems:
* execute `ccmake .` where `assignments.f90` is located on your system, or replace `.` with the path to it if an out-of-source build is preferred, and select the desired Fotran compiler;
* configure by pressing `c` then generate the `makefile` by pressing `g`;
* build with `make`;
* run the generated `assignments` executable.

#### What to Expect:
A text output in the terminal that will verify the aforementioned equations and examples.


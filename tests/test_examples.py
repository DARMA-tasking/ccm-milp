import os
import sys
import unittest
import subprocess
import json
import pulp

from src.ccm_milp.tools import Tools
from src.ccm_milp.generator import CcmMilpGenerator

# Add global path
sys.path.insert(0, os.path.dirname(os.path.join(os.path.dirname(__file__), "../..")))
Examples = Tools.import_class("examples.configuration", "Examples")

class TestExamples(unittest.TestCase):
    """Class to run tests on examples"""

    # Folder paths
    root_dir = os.path.join(os.path.dirname(__file__), "..")
    src_dir = os.path.join(os.path.dirname(__file__), "..", "src")
    config_dir = os.path.join(os.path.dirname(__file__), "config")

    def test_problem_solver(self):
        """Test all examples"""
        # Available CCM-MILP examples
        avail_examples = Examples.list()
        for example_id, example_config in enumerate(avail_examples):
            # Only verified example
            if example_config.test is True:

                # For each different config available
                for _, test_config in enumerate(example_config.test_configs):
                    print()
                    print("-----------------------------------------")
                    print(f"Example : #{example_id}: {example_config.filename}.{example_config.classname}")
                    print("-----------------------------------------")

                    # run ccm_milp_problem for example
                    config_file = os.path.join(self.config_dir, test_config.get("file"))
                    self.assertTrue(os.path.isfile(config_file), f"File: {config_file} does not exist!")

                    print("  Config file: ", test_config.get("file"))
                    print("  ----------------")
                    print("")

                    print("  Generate problem")
                    print("  ----------------")
                    print("")

                    subprocess.run([
                        "python",
                        os.path.join(self.src_dir, "ccm_milp_problem.py"),
                        "-c",
                        config_file
                    ], check=True)

                    # # Check problem file was generated
                    lp_file = os.path.join(self.src_dir, "problem.lp")
                    self.assertTrue(os.path.isfile(lp_file), f"File: {lp_file} does not exist!")
                    mps_file = os.path.join(self.src_dir, "problem.mps")
                    self.assertTrue(os.path.isfile(mps_file), f"File: {mps_file} does not exist!")

                    print("")
                    print("  Solve problem")
                    print("  ----------------")

                    # For each solver available
                    for solver_name in pulp.listSolvers(onlyAvailable=True):
                        print("")
                        print(f"    {solver_name}")
                        print("    ----------------")

                        # Run the solver on the example problem
                        subprocess.run([
                            "python",
                            os.path.join(self.src_dir, "ccm_milp_solver.py"),
                            "-s",
                            solver_name
                        ], check = True)

                        # clear useless generated files
                        mps_sol_file = os.path.join(self.root_dir, "CCM_MILP-pulp.mps")
                        lp_sol_file = os.path.join(self.root_dir, "CCM_MILP-pulp.lp")
                        if os.path.isfile(mps_sol_file):
                            os.remove(mps_sol_file)
                        if os.path.isfile(lp_sol_file):
                            os.remove(lp_sol_file)

                        # Check sol file
                        sol_file = os.path.join(self.root_dir, "CCM_MILP-pulp.sol")
                        self.assertTrue(os.path.isfile(sol_file), f"File: {sol_file} does not exist!")

                        # Readd the sol file content
                        output_str = ""
                        with open(sol_file, 'r', encoding="utf-8") as sol_content:
                            output_str = sol_content.read()
                        print(output_str)

                        # Check sol file content
                        found = False
                        for reg in test_config.get("regexp"):
                            if reg in output_str:
                                print(f"Found {reg}")
                                found = True

                        # No Regexp found
                        if found is False:
                            for reg in test_config.get("regexp"):
                                self.fail("Regex: not found in .sol")

                        # clear .sol file
                        os.remove(sol_file)

                    # Clear created files
                    os.remove(lp_file)
                    os.remove(mps_file)

    def test_permutation(self):
        """Test permutation """
        # Get example SyntheticBlock (id: 1)
        avail_examples = Examples.list()
        synthetic_example = avail_examples[1]

        # Files
        output_file_prefix = "test_permuted_"
        permutation_file   = os.path.join(self.config_dir, "permutation.json")

        # Test all files exists
        self.assertTrue(os.path.isfile(permutation_file), f"File: {permutation_file} does not exist!")
        for input_json_file in synthetic_example.json:
            self.assertTrue(os.path.isfile(input_json_file), f"File: {input_json_file} does not exist!")

        # Command line
        subprocess.run([
            "python",
            os.path.join(self.src_dir, "ccm_milp_permute_json.py"),
            "--permutation-file=" + permutation_file,
            "--input-json-files=" + " ".join(synthetic_example.json),
            "--output-file-prefix=" + output_file_prefix
        ], check=True)

        permutations = []
        with open(permutation_file, 'r', encoding="utf-8") as f:
            permutations = json.load(f)

        tasks_by_rank = {}
        for permutation in permutations:
            if permutation not in tasks_by_rank:
                tasks_by_rank[permutation] = 1
            else:
                tasks_by_rank[permutation] += 1

        # Output files expected
        output_json_files = []
        for input_json_file in synthetic_example.json:
            output_json_files.append(os.path.join(
                CcmMilpGenerator.output_dir(),
                output_file_prefix + input_json_file.split("/").pop()
            ))

        # Check files exists
        for output_json_file in output_json_files:
            # Check file exists
            self.assertTrue(os.path.isfile(output_json_file), f"File: {output_json_file} does not exist!")

            # Get the rank from filename
            rank: int = -1
            if len(output_json_file.split(".")) > 1:
                split_output_json_filename =  output_json_file.split(".")
                rank = int(split_output_json_filename[len(split_output_json_filename) - 2])

            # Load output json
            output_json = {}
            with open(output_json_file, 'r', encoding="utf-8") as f:
                output_json = json.load(f)

            # Get the number of tasks expected for the rank
            number_of_tasks = len(output_json["phases"][1]["tasks"])

            # Check that the number of tasks expected for the rank is correct
            print(f"Rank {rank} expected {tasks_by_rank[rank]} task(s), found {number_of_tasks}" )
            self.assertTrue(
                len(output_json["phases"][1]["tasks"]) == tasks_by_rank[rank],
                f"File: {output_json_file} does not exist!"
            )

            # CLean generated files
            os.remove(output_json_file)

    def test_parse_json(self):
        """Test the JSON parser"""

        # Get example SyntheticBlock (id: 1)
        avail_examples = Examples.list()
        synthetic_example = avail_examples[1]

        # Parse json data files
        data = CcmMilpGenerator.parse_json(synthetic_example.json)

        # Check result
        self.assertEqual(
            len(data.rank_mems), 4,
            f"Bad value for the size of rank_mems: \
                {len(data.rank_mems)} ({data.rank_mems})"
        )
        self.assertEqual(
            len(data.rank_working_bytes), 4,
            f"Bad value for the size of rank_working_bytes: \
                {len(data.rank_working_bytes)} ({data.rank_working_bytes})"
        )
        self.assertEqual(
            len(data.task_loads), 9,
            f"Bad value for the size of task_loads: \
                {len(data.task_loads)} ({data.task_loads})"
        )
        self.assertEqual(
            len(data.task_working_bytes), 9,
            f"Bad value for the size of task_working_bytes: \
                {len(data.task_working_bytes)} ({data.task_working_bytes})"
        )
        self.assertEqual(
            len(data.task_footprint_bytes), 9,
            f"Bad value for the size of task_footprint_bytes: \
                {len(data.task_footprint_bytes)} ({data.task_footprint_bytes})"
        )
        self.assertEqual(
            len(data.task_rank), 9,
            f"Bad value for the size of task_rank: \
                {len(data.task_rank)} ({data.task_rank})"
        )
        self.assertEqual(
            len(data.task_id), 9,
            f"Bad value for the size of task_id: \
                {len(data.task_id)} ({data.task_id})"
        )
        self.assertEqual(
            len(data.memory_blocks), 5,
            f"Bad value for the size of memory_blocks: \
                {len(data.memory_blocks)} ({data.memory_blocks})"
        )
        self.assertEqual(
            len(data.memory_block_home), 5,
            f"Bad value for the size of memory_block_home: \
                {len(data.memory_block_home)} ({data.memory_block_home})"
        )
        self.assertEqual(
            len(data.task_memory_block_mapping), 5,
            f"Bad value for the size of task_memory_block_mapping: \
                {len(data.task_memory_block_mapping)} ({data.task_memory_block_mapping})"
        )
        self.assertEqual(
            len(data.task_communications), 8,
            f"Bad value for the size of task_communications: \
                {len(data.task_communications)} ({data.task_communications})"
        )

    def setUp(self):
        return

    def tearDown(self):
        return

if __name__ == "__main__":
    unittest.main()

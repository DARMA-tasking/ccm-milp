import os
import subprocess
import sys
import unittest


class TestCli(unittest.TestCase):
    root_dir = os.path.join(os.path.dirname(__file__), "..")
    src_dir = os.path.join(root_dir, "src")
    cli_file = os.path.join(src_dir, "ccm_milp_cli.py")

    def test_list_examples(self):
        result = subprocess.run(
            ["python", self.cli_file, "list-examples"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("synthetic_blocks.SyntheticBlocks", result.stdout)

    def test_generate_only_from_config(self):
        config_file = os.path.join(self.root_dir, "tests", "config", "1-load-only.yaml")
        lp_file = os.path.join(self.src_dir, "problem.lp")
        mps_file = os.path.join(self.src_dir, "problem.mps")

        subprocess.run(
            ["python", self.cli_file, "run", "--config", config_file, "--generate-only"],
            check=True,
        )

        self.assertTrue(os.path.isfile(lp_file), f"File does not exist: {lp_file}")
        self.assertTrue(os.path.isfile(mps_file), f"File does not exist: {mps_file}")

        os.remove(lp_file)
        os.remove(mps_file)

    def test_generate_only_from_file_stem(self):
        file_stem = os.path.join(
            self.root_dir,
            "data",
            "synthetic_blocks",
            "synthetic-dataset-blocks",
        )
        lp_file = os.path.join(self.src_dir, "problem.lp")
        mps_file = os.path.join(self.src_dir, "problem.mps")

        subprocess.run(
            ["python", self.cli_file, "run", "--file-stem", file_stem, "--generate-only"],
            check=True,
        )

        self.assertTrue(os.path.isfile(lp_file), f"File does not exist: {lp_file}")
        self.assertTrue(os.path.isfile(mps_file), f"File does not exist: {mps_file}")

        os.remove(lp_file)
        os.remove(mps_file)


if __name__ == "__main__":
    unittest.main()
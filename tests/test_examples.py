import os
import json
import tempfile
import unittest
import pulp

from ccm_milp.configuration import Configuration
from ccm_milp.data import load_rank_files, rank_of_file
from ccm_milp.generator import Generator
from ccm_milp.permute import permute

ROOT_DIR = os.path.join(os.path.dirname(__file__), "..")
DATA_DIR = os.path.join(ROOT_DIR, "data")
CONFIG_DIR = os.path.join(os.path.dirname(__file__), "config")

# (parameter file, dataset directory, expected objective value)
SOLVE_CASES = [
    ("1-load-only.yaml", "synthetic_blocks", 2.0),
    ("1-load-only-and-cluster.yaml", "synthetic_blocks", 2.5),
    ("1-load-only-and-memory-bound.yaml", "synthetic_blocks", 2.0),
    ("1-fwmp-with-alpha.yaml", "synthetic_blocks", 2.0),
    ("1-fwmp-with-alpha-beta.yaml", "synthetic_blocks", 4.0),
    ("1-load-no-memory-homing-01.yaml", "synthetic_blocks", 2.5),
    ("1-load-no-memory-homing-03.yaml", "synthetic_blocks", 4.0),
    ("2-null-case.yaml", "synthetic_blocks_alpha0", 0.0),
    ("2-off-node-communication-only.yaml", "synthetic_blocks_alpha0", 0.0),
    ("3-load-only.yaml", "synthetic_blocks_bad_rank0", 3.0),
]

class TestExamples(unittest.TestCase):
    """Class to run tests on examples"""

    def test_problem_solver(self):
        """Solve each test case with every available solver and check the objective"""
        for config_file, dataset, expected in SOLVE_CASES:
            config = Configuration.from_yaml(os.path.join(CONFIG_DIR, config_file))
            data = load_rank_files([os.path.join(DATA_DIR, dataset)], config)
            for solver_name in pulp.listSolvers(onlyAvailable=True):
                with self.subTest(config=config_file, solver=solver_name):
                    generator = Generator(config, data)
                    generator.generate_problem()
                    solution = generator.solve(solver_name)
                    self.assertTrue(solution.is_optimal, solution.status)
                    self.assertAlmostEqual(solution.objective, expected, places=6)

    def test_permutation(self):
        """Permuted rank files hold the tasks assigned to each rank"""
        permutation_file = os.path.join(CONFIG_DIR, "permutation.json")
        with open(permutation_file, 'r', encoding="utf-8") as f:
            permutation = json.load(f)

        with tempfile.TemporaryDirectory() as output_dir:
            output_files = permute(
                permutation_file, [os.path.join(DATA_DIR, "synthetic_blocks")], output_dir)
            self.assertEqual(len(output_files), 4)
            for output_file in output_files:
                rank = rank_of_file(output_file)
                with open(output_file, 'r', encoding="utf-8") as f:
                    output_json = json.load(f)
                self.assertEqual(len(output_json["phases"][1]["tasks"]), permutation.count(rank), output_file)

    def test_parse_json(self):
        """Test the JSON parser"""
        data = load_rank_files([os.path.join(DATA_DIR, "synthetic_blocks")], Configuration())
        expected_sizes = {
            "rank_mems": 4,
            "rank_working_bytes": 4,
            "task_loads": 9,
            "task_working_bytes": 9,
            "task_footprint_bytes": 9,
            "task_id": 9,
            "memory_blocks": 5,
            "memory_block_home": 5,
            "task_memory_block_mapping": 5,
            "task_communications": 8,
        }
        for name, size in expected_sizes.items():
            self.assertEqual(len(getattr(data, name)), size, f"{name}: {getattr(data, name)}")

def write_rank_files(directory: str, ranks: list):
    """Write one rank file per (tasks, communications) pair, with tasks given as (id, load, shared_id, shared_bytes)"""
    def entity(obj_id):
        return {"type": "object", "id": obj_id}
    for rank, (tasks, communications) in enumerate(ranks):
        phase = {
            "id": 0,
            "tasks": [{
                "entity": entity(obj_id),
                "time": load,
                "user_defined": {"shared_id": shared_id, "shared_bytes": shared_bytes}
            } for obj_id, load, shared_id, shared_bytes in tasks],
            "communications": [
                {"from": entity(src), "to": entity(dst), "bytes": size} for src, dst, size in communications]}
        with open(os.path.join(directory, f"test.{rank}.json"), 'w', encoding="utf-8") as f:
            json.dump({"metadata": {"rank": rank}, "phases": [phase]}, f)

class TestDataParsing(unittest.TestCase):
    """Regression tests for parsing bugs that silently produced wrong problems"""

    def test_communications_use_task_indices(self):
        """Communication endpoints are entity ids, which must be mapped to task indices"""
        with tempfile.TemporaryDirectory() as data_dir:
            write_rank_files(data_dir, [
                ([(2883587, 1.0, 0, 1.0), (1048579, 1.0, 0, 1.0)], [(2883587, 524291, 8.0)]),
                ([(524291, 1.0, 1, 1.0)], [(524291, 1048579, 4.0)]),
            ])
            data = load_rank_files([data_dir], Configuration())
        self.assertEqual(data.task_communications, [[0, 2, 8.0], [2, 1, 4.0]])

    def test_unknown_communication_task_is_rejected(self):
        """A communication to a task that does not exist is an error"""
        with tempfile.TemporaryDirectory() as data_dir:
            write_rank_files(data_dir, [([(10, 1.0, 0, 1.0)], [(10, 99, 1.0)])])
            with self.assertRaisesRegex(ValueError, "unknown task id 99"):
                load_rank_files([data_dir], Configuration())

    def test_block_arrays_stay_aligned(self):
        """Block sizes, homes, and task mappings describe the same block at each index"""
        with tempfile.TemporaryDirectory() as data_dir:
            write_rank_files(data_dir, [
                ([(0, 1.0, 1, 9.0)], []),
                ([(1, 1.0, 2, 5.0)], []),
            ])
            data = load_rank_files([data_dir], Configuration())
        blocks = set(zip(data.memory_blocks, data.memory_block_home, map(tuple, data.task_memory_block_mapping)))
        self.assertEqual(blocks, {(9.0, 0, (0,)), (5.0, 1, (1,))})

if __name__ == "__main__":
    unittest.main()

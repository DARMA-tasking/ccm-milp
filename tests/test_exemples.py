import subprocess
import os
import unittest
import pulp

from src.CcmMilp.Configuration import Examples

class TestExamples(unittest.TestCase):
    """Class to run tests on examples"""

    def setUp(self):
        return

    def tearDown(self):
        return

    def test_all_examples(self):
        """Test all examples"""
        # Get src dir
        root_dir = os.path.join(os.path.dirname(__file__), '..')
        src_dir = os.path.join(root_dir, 'src')

        # Available CCM-MILP examples
        avail_examples = Examples.list()
        for example_id, example_config in enumerate(avail_examples):
            # Only verified example
            if example_config.test is True:

                # For each different config available
                for _, test_config in enumerate(example_config.test_configs):
                    print()
                    print('-----------------------------------------')
                    print(f'Example : #{example_id}: {example_config.filename}.{example_config.classname}')
                    print('-----------------------------------------')

                    # run ccm_milp_problem for example
                    config_file = os.path.join(os.path.dirname(__file__), 'config', test_config.get('file'))
                    self.assertTrue(os.path.isfile(config_file), f'File: {config_file} does not exist!')

                    print('  Config file: ', test_config.get('file'))
                    print('  ----------------')
                    print('')

                    print('  Generate problem')
                    print('  ----------------')
                    print('')

                    subprocess.run([
                        'python',
                        os.path.join(src_dir, 'ccm_milp_problem.py'),
                        '-c',
                        config_file
                    ], check=True)

                    # # Check problem file was generated
                    lp_file = os.path.join(src_dir, 'problem.lp')
                    self.assertTrue(os.path.isfile(lp_file), f'File: {lp_file} does not exist!')
                    mps_file = os.path.join(src_dir, 'problem.mps')
                    self.assertTrue(os.path.isfile(mps_file), f'File: {mps_file} does not exist!')

                    print('')
                    print('  Solve problem')
                    print('  ----------------')

                    # For each solver available
                    for solver_name in pulp.listSolvers(onlyAvailable=True):
                        print('')
                        print(f'    {solver_name}')
                        print('    ----------------')

                        # Run the solver on the example problem
                        subprocess.run([
                            'python',
                            os.path.join(src_dir, 'ccm_milp_solver.py'),
                            '-s',
                            solver_name
                        ], check = True)

                        # clear useless generated files
                        mps_sol_file = os.path.join(root_dir, 'CCM_MILP-pulp.mps')
                        lp_sol_file = os.path.join(root_dir, 'CCM_MILP-pulp.lp')
                        if os.path.isfile(mps_sol_file):
                            os.remove(mps_sol_file)
                        if os.path.isfile(lp_sol_file):
                            os.remove(lp_sol_file)

                        # Check sol file
                        sol_file = os.path.join(root_dir, 'CCM_MILP-pulp.sol')
                        self.assertTrue(os.path.isfile(sol_file), f'File: {sol_file} does not exist!')

                        # Readd the sol file content
                        output_str = ''
                        with open(sol_file, 'r', encoding="utf-8") as sol_content:
                            output_str = sol_content.read()
                        print(output_str)

                        # Check sol file content
                        found = False
                        for reg in test_config.get('regexp'):
                            if reg in output_str:
                                print(f"Found {reg}")
                                found = True

                        # No Regexp found
                        if found is False:
                            for reg in test_config.get('regexp'):
                                self.fail('Regex: not found in .sol')

                        # clear .sol file
                        os.remove(sol_file)

                    # Clear created files
                    os.remove(lp_file)
                    os.remove(mps_file)

if __name__ == '__main__':
    unittest.main()

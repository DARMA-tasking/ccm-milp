import os
import subprocess
import unittest


class TestExemple1(unittest.TestCase):
    """Class to run acceptance tests"""

    def setUp(self):
        return

    def tearDown(self):
        return

    def test_acceptance(self):
        # # Prepare log file
        # log_file = os.path.join(os.path.dirname(__file__), "output", "log1.txt")
        # self.assertTrue(os.path.isfile(log_file), f"File: {log_file} does not exist!")
        
        # run ccm_milp_problem for exemple 1
        config_file = os.path.join(os.path.dirname(__file__), "config", "example_1.yaml")
        subprocess.run(["python", "src/ccm_milp_problem.py", "-c", config_file], check=True)
        
        
        # # run ccm_milp_solver for exemple 1 with CBC
        # subprocess.run(["python", "ccm_milp_solver.py"], check=True)
        
        # Create log file        

        # # validate log file content
        # with open(log_file, 'r', encoding="utf-8") as logger_output:
        #     output_str = logger_output.read()
        #     regex_list = [
        #         "Optimal - objective value 2.0",
        #         "Objective:  OBJ = 2 (MINimum)",
        #     ]
            
        #     # Init found
        #     found = False
        #     for reg in regex_list:
        #         if reg in output_str:
        #             print(f"Found {reg}")
        #             found = True

        #     # No Regexp found
        #     if (found == False):
        #         for reg in regex_list:
        #             self.fail(f"Regex: {reg} not found in log.\nTEST FAILED.")

if __name__ == "__main__":
    unittest.main()

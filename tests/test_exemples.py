import os
import unittest
import pulp
import subprocess
import shutil

from src.modules.configuration import Examples

"""Class to run tests on exemples"""
class TestExemples(unittest.TestCase):

    def setUp(self):
        return

    def tearDown(self):
        return

    def test_all_exemples(self):
        # Get src dir
        rootDir = os.path.join(os.path.dirname(__file__), '..')
        srcDir = os.path.join(rootDir, 'src')
        
        # Available CCM-MILP examples
        avail_examples = Examples.list()
    
        for i, [moduleName, className, regexpTest] in enumerate(avail_examples):

            print()
            print('-----------------------------------------')
            print(f'Exemple : #{i}: {moduleName}.{className}')
            print('-----------------------------------------')
    
            print('  Generate problem')
            print('  ----------------')
            print('')
            
            # run ccm_milp_problem for exemple 1
            config_file = os.path.join(os.path.dirname(__file__), 'config', f'example_{i}.yaml')
            self.assertTrue(os.path.isfile(config_file), f'File: {config_file} does not exist!')
            
            subprocess.run(['python', os.path.join(srcDir, 'ccm_milp_problem.py'), '-c', config_file], check=True)
            
            # # Check problem file was generated
            lpFile = os.path.join(srcDir, 'problem.lp')
            self.assertTrue(os.path.isfile(lpFile), f'File: {lpFile} does not exist!')
            mpsFile = os.path.join(srcDir, 'problem.mps')
            self.assertTrue(os.path.isfile(mpsFile), f'File: {mpsFile} does not exist!')
            
            print('')
            print('  Solve problem')
            print('  ----------------')
            
            # For each solver available
            for solverName in pulp.listSolvers(onlyAvailable=True):
                print('')
                print(f'    {solverName}')
                print('    ----------------')
                
                # Run the solver on the example problem 
                subprocess.run(['python', os.path.join(srcDir, 'ccm_milp_solver.py'), '-s', solverName], check=True)
        
                # clear useless generated files
                mpsSolFile = os.path.join(rootDir, 'CCM_MILP-pulp.mps')
                lpSolFile = os.path.join(rootDir, 'CCM_MILP-pulp.lp')
                if (os.path.isfile(mpsSolFile)): os.remove(mpsSolFile)
                if (os.path.isfile(lpSolFile)): os.remove(lpSolFile)
        
                # Check sol file
                solFile = os.path.join(rootDir, 'CCM_MILP-pulp.sol')
                self.assertTrue(os.path.isfile(solFile), f'File: {solFile} does not exist!')
                
                # solFile2 = os.path.join(rootDir,'sol', moduleName, solverName, 'CCM_MILP-pulp.sol')
                # os.makedirs(os.path.dirname(solFile2))
                # shutil.copy(solFile, solFile2)
                         
                # Readd the sol file content
                output_str = ''
                with open(solFile, 'r', encoding="utf-8") as solContent:
                     output_str = solContent.read()
                
                print(output_str)
                     
                # Check sol file content
                found = False
                for reg in regexpTest:
                    if reg in output_str:
                        print(f"Found {reg}")
                        found = True

                # No Regexp found
                if (found == False):
                    for reg in regexpTest:
                        self.fail(f"Regex: not found in .sol.")

    
                # clear .sol file
                os.remove(solFile)
            
            # Clear created files
            os.remove(lpFile)
            os.remove(mpsFile)

if __name__ == '__main__':
    unittest.main()
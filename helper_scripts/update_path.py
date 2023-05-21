import os
import pathlib

curr_path = pathlib.Path(os.path.dirname(__file__))  # Path to this script

run_file_path: pathlib.Path = pathlib.Path(curr_path.parent, "code/run.py")  # Path to code/run.py

with open(run_file_path, "w") as f:
    # Write the relative path to the code folder to the run.py file, append it to path, and import the main script
    f.write(f"import sys\n"
            f"code_path: str = r'{run_file_path.parent}'\n"
            f"sys.path.append(code_path)\n"
            f"import hello_world  # This script will run when imported, thus starting the program\n")

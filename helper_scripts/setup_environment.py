import os
import sys
from pathlib import Path

print("Make sure that Webots uses the same Python environment as this project:\n"
      "1. Open Webots\n"
      "2. Click Tools -> Preferences\n"
      f"3. Enter '{sys.executable}' (without quotes) in 'Python command:' field\n")

webots = Path(os.environ.get("WEBOTS_HOME") or "<WEBOTS_HOME directory>").joinpath("lib", "controller", "python")
print("Add Webots Python modules to this project to prevent unresolved reference errors:\n"
      "1. In PyCharm, open Settings...\n"
      "2. Navigate to Project: ... -> Project Structure\n"
      f"3. Click + Add Content Root, enter '{webots}' (without quotes) and press OK\n")

# Cannot use `Path(__file__).parent` in the run.py file, because it is saved under different location
# when loaded from the Robot Window. Plus, if that would be the case, relative import would work anyway.
main_script = "hello_world.py"
main_script = Path(__file__).parent.joinpath("..", "code", main_script).resolve()
main_script.parent.joinpath("run.py").write_text(
    "import sys\n"
    "from pathlib import Path\n\n"
    f"main_script = Path(r'{main_script}')\n\n"
    f"sys.path.append(main_script.parent.as_posix())\n\n"
    "print(f'\\nRunning {main_script}\\n')\n\n"
    f"# Execute the main script instead of importing it, to ensure its `__name__ == '__main__'`\n"
    f"exec(main_script.read_text())\n"
)

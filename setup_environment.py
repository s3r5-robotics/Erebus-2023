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

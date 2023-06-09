import sys
from pathlib import Path

main_script = Path(r'C:\Programming\RoboCup_Erebus\Erebus-2023\code\main.py')

sys.path.append(main_script.parent.as_posix())

print(f'\nRunning {main_script}\n')

# Execute the main script instead of importing it, to ensure its `__name__ == '__main__'`
exec(main_script.read_text())

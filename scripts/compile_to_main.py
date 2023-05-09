import os


# List of files to compile in order
# One file's missing!!!
files: list[str] = [
    "run.py",
    "utilities.py",
    "flags.py",
    "data_structures/angle.py",
    "data_structures/compound_expandable_grid.py",
    "flow_control/sequencer.py",
    "flow_control/state_machine.py",
    "flow_control/delay.py",
    "data_structures/vectors.py",
    "executor/stuck_detector.py",
    "flow_control/step_counter.py",
    "robot/devices/wheel.py",
    "robot/devices/sensor.py",
    "robot/devices/camera.py",
    "robot/devices/lidar.py",
    "robot/devices/gps.py",
    "robot/devices/gyroscope.py",
    "robot/devices/comunicator.py",
    "robot/pose_manager.py",
    "robot/drive_base.py",
    "robot/robot.py",
    "data_structures/compound_pixel_grid.py",
    "data_structures/tile_color_grid.py",
    "mapping/wall_mapper.py",
    "mapping/floor_mapper.py",
    "mapping/robot_mapper.py",
    "mapping/data_extractor.py",
    "fixture_detection/color_filter.py",
    "fixture_detection/fixture_detection.py",
    "mapping/mapper.py",
    "agents/agent.py",
    "algorithms/np_bool_array/efficient_a_star.py",
    "algorithms/np_bool_array/bfs.py",
    "algorithms/np_bool_array/a_star.py",
    "agents/granular_navigation_agent/path_smoothing.py",
    "agents/granular_navigation_agent/path_finder.py",
    "agents/granular_navigation_agent/best_position_finder.py",
    "agents/granular_navigation_agent/victim_position_finder.py",
    "agents/granular_navigation_agent/granular_navigation_agent.py",
    "fixture_detection/victim_clasification.py",
    "fixture_detection/fixture_clasification.py",
    "executor/executor.py",
    "main.py",
]

output: str = ""
for file in files:
    output += f"\n\n\n#######################\n# FILE: {file}\n#######################\n\n"
    with open(os.path.join("../src", file)) as f:
        for line in f:
            if len(line.split()) and (line.split()[0] == "from") and (not file == "utilities.py"):
                # In all files but `utilities.py`, remove all `from` imports
                continue
            output += line
    print(f"Added file: `{file}`")

with open("../main.py", "w") as f:
    f.write(output)

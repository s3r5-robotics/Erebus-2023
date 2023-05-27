# RoboCup Junior Simulation - Erebus

## Initial Installation ([Erebus - Installation](https://erebus.rcj.cloud/docs/installation/]))

1. Download and install the simulation platform from\
   https://cyberbotics.com/

2. Initialize or update the Erebus submodule by executing the following command in this directory
   ```shell
   git submodule update --init --progress
   ```
   Optionally, you can check for new Erebus release using
   ```shell
   cd erebus
   # List all release tags, sorted by date
   git tag --sort=committerdate --list 'v*'
   # Checkout the latest release tag (e.g. 'v23.0.4' in time of writing)
   git checkout v23.0.4
   ```
   This is normally not required, as the submodule is committed with specific hash anyway, so the command
   in the previous step checks-out the correct specific version. This is useful as an occasional check for
   new releases, to be checked out and committed as a new submodule hash.

3. Setup virtual environment based on Pipfile (using PyCharm or by executing `pipenv install` in this directory)

4. Run script [`helper_scripts/setup_environment.py`](helper_scripts/setup_environment.py) using this virtual environment and follow
   its further instructions.

## Running the Simulation

1. Open one of the `.wbt` worlds located in [`erebus/game/worlds/`](erebus/game/worlds) in Webots

2. If Erebus Simulation Control web page did open automatically in the default browser, then
   in the left panel, right-click on `DEF MAINSUPERVISOR Robot` and click _Show Robot Window_.

3. Click "**LOAD**" under Program icon
   (<img src="erebus/game/plugins/robot_windows/MainSupervisorWindow/program.png" width="20" height="20">)
   and load the [`run.py`](code/run.py) script.

4. Click "**LOAD**" under robot icon
   (<img src="erebus/game/plugins/robot_windows/MainSupervisorWindow/robot.png" width="20" height="20">)
   and load the [`{robot}.json`](robots).

5. Click the "***Play button***" to start the simulation.  

As long as you don't move the `code/` folder, you can just press the "**Reset**" and "**Play**" buttons, 
to run the simulation again - `run.py` will automatically reload `main.py`.

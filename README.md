# RoboCup Junior Simulation - Erebus

## Initial Installation ([Erebus - Installation](https://erebus.rcj.cloud/docs/installation/]))

1. Download and install the simulation platform from\
   https://cyberbotics.com/

2. Initialize or update the Erebus submodule by executing the following command in this directory
   ```shell
   git submodule update --init --progress
   ```

3. Optionally, you can check for new Erebus release using
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

## Running the Simulation

1. Open one of the `.wbt` worlds located in [`erebus/game/worlds/`](erebus/game/worlds/) in Webots

2. In Webots, in the left panel, right-click on `DEF MAINSUPERVISOR Robot` and click _Show Robot Window_.\
This should open Erebus Simulation Control web page in the default browser.

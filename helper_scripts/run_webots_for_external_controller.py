import datetime
import json
import re
import subprocess
import threading
import time
from pathlib import Path
from typing import Tuple, Optional, Set, Union, List

import psutil
import websockets.sync.client as ws_client

import controller
from helper_scripts.setup_environment import get_webots_executable_path


def _find_existing_proces() -> Optional[psutil.Process]:
    # On Windows, the main GUI process is "webotsw.exe", the command line process is "webots.exe".
    # On Linux, the main GUI process is "webots".
    # On all platforms, "webots-bin" is started as a child process of the main GUI process,
    # and it is the one that actually runs the simulation.
    for p in psutil.process_iter():
        if "webots-bin" in p.name():
            return p
    return None


def _start_new_process(exe_path: Path, world_file: Optional[Path], fast: bool = False) -> subprocess.Popen:
    # https://cyberbotics.com/doc/guide/starting-webots#command-line-arguments
    webots_args = [
        exe_path,
        # Webots binds the socket (server) in such a way that we cannot check if it is already in use using
        # available methods from the socket module (and chose the next free port before starting new Webots
        # instance). psutil module is used to check the used ports after starting the process.
        "--port=1234",  # Set the TCP port number to use for the Webots (Robot Window) server
        "--minimize",  # Minimize the Webots window on startup
        "--batch",  # Prevent Webots from creating blocking pop-up windows
        "--stdout",  # Redirect the stdout of the controllers to the terminal - required to catch the port number
        "--stderr",  # Redirect the stderr of the controllers to the terminal - required to catch the port number
    ]
    if fast:
        webots_args.append("--mode=fast")  # Choose the startup mode (pause / realtime / fast)
    if world_file:
        webots_args.append(world_file.resolve())

    # Start the Webots and load the given world file
    # (psutil.Popen is used instead of subprocess.Popen to combine functionality of both classes into one class)
    return psutil.Popen(webots_args, text=True)


def _get_process_connections(process: psutil.Process, min_conns: int, timeout: float) -> Set['psutil._common.pconn']:
    kind = "tcp4"
    t0 = time.perf_counter()
    conns = set()

    def filter_fn(c: 'psutil._common.pconn') -> bool:
        return c.status == psutil.CONN_LISTEN

    while True:
        conns.update(filter(filter_fn, process.connections(kind)))
        for cp in process.children(recursive=True):
            # There is a chance that process exits before we can get its connections.
            try:
                conns.update(filter(filter_fn, cp.connections(kind)))
            except psutil.NoSuchProcess:
                pass

        if len(conns) >= min_conns:
            break
        if time.perf_counter() - t0 > timeout:
            if process.status() != psutil.STATUS_RUNNING:
                raise TimeoutError(f"'{process.exe()}' process did not start in {timeout} s")
            else:
                raise TimeoutError(f"'{process.exe()}' did not setup server in {timeout} s")

    return conns


def run_webots(world_file: Optional[Union[str, Path]] = None, force_new_instance: bool = False, timeout: float = 20) \
        -> Tuple[subprocess.Popen, int]:
    """
    Get Webots process and port number of the Robot Window server

    :param world_file:         Path to - or name of - the world file to load in Webots, if new instance is started.
    :param force_new_instance: If True, start a new Webots instance even if one is already running.
                               If False, try to reuse any running Webots instance.
    :param timeout:            Timeout in seconds for starting Webots and checking for the controller server.

    :return: Webots process, port number of the Supervisor server.
    """
    p = None if force_new_instance else _find_existing_proces()
    if p:
        print(f"Reusing existing Webots instance (PID: {p.pid},"
              f" started {datetime.datetime.fromtimestamp(p.create_time()):%H:%M:%S})")
    else:
        exe_path = get_webots_executable_path()

        if world_file and (not Path(world_file).is_file()):
            # This is not an absolute path, so it is probably a world file in the default directory
            world_file = Path(__file__).parent.joinpath("..", "erebus", "game", "worlds", world_file)

        p = _start_new_process(exe_path, world_file)

        print(f"Created new Webots instance (PID: {p.pid})")

    # Webots UI starts another process which handles the simulation and the controllers,
    # meaning that there is some delay until connections are established (starting the
    # UI process, starting the child processes, starting the Python Supervisor).
    # We know at least 2 connections are required, one for Robot Window (web page and
    # Websocket for sending command from web page JavasCript), another for uploading
    # the controller code and robot JSON (server created in ControllerUploader.py).
    conns = _get_process_connections(p, min_conns=2, timeout=timeout)

    # Lower port is usually the Supervisor server (1234 by default), higher port is usually the
    # server for the files by ControllerUploader.py (60520).
    ports = sorted(c.laddr.port for c in conns)
    print(f"Webots uses port {ports[0]} for the Supervisor server and port {ports[1]} for file upload")

    return p, ports[0]


def _ws_readall(wsc: ws_client.ClientConnection, robot_name: Optional[str]) -> List[str]:
    """
    Get list of all messages already received from the Websocket connection

    :param wsc:        Websocket connection.
    :param robot_name: If not None, only messages for this robot are returned ("" to get messages for all robots).
                       If None, return all other (stdout, stderr) messages.
    """
    messages = []
    while True:
        try:
            msg = wsc.recv(timeout=0.1)
        except TimeoutError:
            break
        if msg.startswith("robot: "):
            if robot_name is not None:
                jsn = json.loads(msg.split(":", 1)[1])
                name = jsn["name"]
                msg = jsn["message"]
                if (robot_name == "") or (name == robot_name):
                    messages.append(msg)
                else:
                    print(f"Received message for robot '{name}': {msg}")
        elif robot_name is None:
            messages.append(msg)
    return messages


def enable_extern_controller(server_port: int, robot_name: str = "robot", timeout: float = 1) -> Tuple[str, str, str]:
    """
    Connect to Webots Supervisor server and enable extern controller

    :return: External robot name, local IPC address, local TCP address.
    """
    # This is mostly copied from erebus/game/plugins/robot_windows/MainSupervisorWindow/MainSupervisorWindow.js
    # and https://cyberbotics.com/wwi/R2023a/RobotWindow.js which is imported by MainSupervisorWindow.js.
    with ws_client.connect(f"ws://localhost:{server_port}", open_timeout=timeout) as ws:
        # Reload the Robot Window to get initial state
        ws.send(f"robot:{robot_name}:rw_reload")
        msgs = _ws_readall(ws, robot_name=robot_name)
        if not msgs:
            raise RuntimeError(f"No response to Robot Window Reload ({robot_name}:rw_reload) command")
        msgs.reverse()  # History is saved, parse the latest messages first

        # Find and parse the latest config message ({'message': 'config,1,0,0,0', 'name': 'robot'}`)
        if next(filter(lambda msg: msg.startswith("config"), msgs), ",?").split(",")[1] != "1":
            print("'Keep controller/robot files' option is not enabled"
                  " - Reload World in Webots and enable it in the Robot Window!")

        # Find and parse the latest loaded message - loaded0 means default robot JSON has loaded
        if not next(filter(lambda msg: msg.startswith("loaded"), msgs), "").startswith("loaded1"):
            print("No custom robot JSON loaded"
                  " - Reload World in Webots and load it in the Robot Window!")

        # Enable extern robot controllers
        ws.send(f"robot:{robot_name}:remoteEnable")
        # Start the supervisor to accept external controllers
        ws.send(f"robot:{robot_name}:run")

        # Read all pending messages which were returned as a response to the above commands.
        # We are only interested in one specific message confirming that the extern controller was enabled.
        pattern = re.compile(r".*INFO: '(.+)' extern controller: waiting for connection on (.+) or on (.+)")
        msgs = _ws_readall(ws, robot_name=None)
        for msg in msgs:
            m = pattern.match(msg)
            if m:
                return m.group(1), m.group(2), m.group(3)

        print("Received responses:", msgs)
        raise RuntimeError("Could not enable extern robot controller (no response)")


def check_if_extern_controller_is_enabled(timeout: float = 5) -> bool:
    def _start_controller():
        # This will try to establish connection to the Webots
        _ = controller.Robot()

    # Run a daemon thread which will try to create a Robot instance. If successful,
    # thread will stop and join the main thread. If not successful, thread will be
    # terminated when this script exits (daemon=True).
    t = threading.Thread(target=_start_controller, daemon=True)
    t.start()
    try:
        t.join(timeout=timeout)
    except TimeoutError:
        pass

    return not t.is_alive()


def main():
    webots_process, webots_port = run_webots()
    # Slight delay is required for the server to start accepting connections
    time.sleep(2)
    try:
        robot_name, icp_url, _ = enable_extern_controller(webots_port)
    except RuntimeError as e:
        print(f"Could not send command to enable extern robot controller ({e})", flush=True)

    print("Checking if extern controller can be started...", flush=True)
    if check_if_extern_controller_is_enabled():
        print("Extern Robot Controller is enabled")
    else:
        print("Could not start Extern Robot Controllers - try Reload World command in Webots and try again")

    # TODO: Edit the .run/main.xml to insert WEBOTS_CONTROLLER_URL=<icp_url> environment variable


if __name__ == "__main__":
    main()

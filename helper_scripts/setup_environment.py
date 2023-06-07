import os
import platform
import sys
from pathlib import Path

WB_HOME_VAR = "WEBOTS_HOME"


def ensure_webots_home_environment_variable() -> Path:
    """
    Make sure that WEBOTS_HOME environment variable is set and points to the Webots installation directory

    :return: WEBOTS_HOME path
    """
    path = os.environ.get(WB_HOME_VAR)
    if not path:
        path = Path("C:/Program Files/Webots" if platform.system() == "Windows" else "/usr/local/webots").resolve()
        print(f"{WB_HOME_VAR} environment variable not set, using default '{path}'")
    else:
        path = Path(path).resolve()

    if not path.is_dir():
        raise ValueError(f"{WB_HOME_VAR} directory '{path}' does not exist")

    # Make sure that this path is the correct path to Webots installation directory
    get_webots_executable_path(path)

    # One would be tempted to automatically add the WEBOTS_HOME environment variable
    # using `os.environ[WB_HOME_VAR] = str(path)` here, but that would not work when
    # running Webots, because they are started in a separate process with fresh env.
    # Moreover, WEBOTS_HOME shall be set once during the Webots installation anyway,
    # and never changed unless Webots is reinstalled to a different location.
    if not os.environ.get(WB_HOME_VAR):
        raise EnvironmentError(f"Set {WB_HOME_VAR} environment variable to '{path}' (without quotes) and try again")

    return path


def get_webots_executable_path(webots_home: Path = None) -> Path:
    """
    Get the path to the main Webots command-line executable

    :param webots_home: Path to the main webots executable supporting command line arguments
    """
    if not webots_home:
        webots_home = ensure_webots_home_environment_variable()

    if platform.system() == "Windows":
        wb = webots_home.joinpath("msys64", "mingw64", "bin", "webots.exe")
    else:
        wb = webots_home.joinpath("bin", "webots")

    if not (wb and os.access(wb, os.X_OK)):
        raise EnvironmentError(f"{WB_HOME_VAR} '{webots_home}' does not contain the Webots executable '{wb}'")

    return wb


def _webots_uses_correct_python_executable() -> None:
    print("Make sure that Webots uses the same Python environment as this project:\n"
          "1. Open Webots\n"
          "2. Click Tools -> Preferences\n"
          f"3. Enter '{sys.executable}' (without quotes) in 'Python command:' field\n")


def _webots_home_to_content_roots() -> None:
    try:
        wbh = ensure_webots_home_environment_variable()
    except EnvironmentError as e:
        exit(f"Invalid {WB_HOME_VAR} environment variable - it shall point to the Webots installation directory ({e})")

    wbh = wbh.joinpath("lib", "controller", "python")
    print("Add Webots Python modules to this project to prevent unresolved reference errors:\n"
          "1. In PyCharm, open Settings...\n"
          "2. Navigate to Project: ... -> Project Structure\n"
          f"3. Click + Add Content Root, enter '{wbh}' (without quotes) and press OK\n")


def _generate_run_py_script_with_correct_path(main_script_name: str) -> None:
    # Cannot use `Path(__file__).parent` in the run.py file, because it is saved under different location
    # when loaded from the Robot Window. Plus, if that would be the case, relative import would work anyway.
    repo_root = Path(__file__).parent.parent.resolve()
    main_script = repo_root.joinpath("code", main_script_name).resolve()  # Resolve in case of relative path
    run_script = repo_root.joinpath("code", "run.py")
    run_script.write_text(
        "import sys\n"
        "from pathlib import Path\n\n"
        f"main_script = Path(r'{main_script}')\n\n"
        f"sys.path.append(main_script.parent.as_posix())\n\n"
        "print(f'\\nRunning {main_script}\\n')\n\n"
        f"# Execute the main script instead of importing it, to ensure its `__name__ == '__main__'`\n"
        f"exec(main_script.read_text())\n"
    )

    print(f"Generated '{run_script.relative_to(repo_root)}' to automatically run"
          f" '{main_script.relative_to(repo_root)}' when running simulation on local machine\n")


if __name__ == "__main__":
    print("########## Environment setup ##########\n")
    _webots_uses_correct_python_executable()
    _webots_home_to_content_roots()
    _generate_run_py_script_with_correct_path("main.py")
    print("#######################################")

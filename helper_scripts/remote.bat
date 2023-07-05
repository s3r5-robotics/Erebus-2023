@echo off

set PYTHONPATH=%WEBOTS_HOME%\lib\controller\python

:: 10.0.11.79 - Waldi PC IP
set WEBOTS_CONTROLLER_URL=tcp://10.0.11.79:1234/Erebus_Bot

python ../code/run.py

pause
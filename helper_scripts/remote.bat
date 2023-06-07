@echo off

set PYTHONPATH=%WEBOTS_HOME%\lib\controller\python

set WEBOTS_CONTROLLER_URL=tcp://192.168.10.18:1234/Erebus_Bot

python run.py

pause

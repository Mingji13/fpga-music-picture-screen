@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto dependencies
echo Creating local Python environment...
py -3.12 -m venv .venv
if errorlevel 1 goto failed
:dependencies
".venv\Scripts\python.exe" -c "import numpy" >nul 2>&1
if not errorlevel 1 goto run
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto failed
:run
".venv\Scripts\python.exe" music_sync.py export-cue %*
if errorlevel 1 goto failed
pause
exit /b 0
:failed
echo Export failed. Inputs and existing exports are not replaced.
pause
exit /b 1

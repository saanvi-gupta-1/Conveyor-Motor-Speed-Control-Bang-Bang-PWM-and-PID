@echo off
setlocal
cd /d "%~dp0"

set "PYTHON_EXE="
for /f "delims=" %%P in ('where.exe python.exe 2^>nul') do (
    if not defined PYTHON_EXE (
        "%%P" --version >nul 2>nul
        if not errorlevel 1 set "PYTHON_EXE=%%P"
    )
)
if not defined PYTHON_EXE (
    echo Python 3.10 or newer was not found. Install Python from python.org and try again.
    pause
    exit /b 1
)

if not exist ".venv\Scripts\python.exe" (
    echo Setting up the project environment. This is needed only on first run.
    "%PYTHON_EXE%" -m venv .venv
    if errorlevel 1 goto setup_failed
    ".venv\Scripts\python.exe" -m pip install --upgrade pip
    if errorlevel 1 goto setup_failed
    ".venv\Scripts\python.exe" -m pip install -e ".[dev,visual]"
    if errorlevel 1 goto setup_failed
)

".venv\Scripts\python.exe" -m conveyor_control.cli %*
set "APP_EXIT=%ERRORLEVEL%"
if not "%APP_EXIT%"=="0" pause
exit /b %APP_EXIT%

:setup_failed
echo Setup failed. See the error above and the Installation section in README.md.
pause
exit /b 1

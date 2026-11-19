@echo off
setlocal
cd /d "%~dp0"

set "PYTHON_EXE="
if exist ".venv\Scripts\python.exe" (
    set "PYTHON_EXE=.venv\Scripts\python.exe"
) else (
    for /f "delims=" %%P in ('where.exe python.exe 2^>nul') do (
        if not defined PYTHON_EXE (
            "%%P" --version >nul 2>nul
            if not errorlevel 1 set "PYTHON_EXE=%%P"
        )
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

rem If command line arguments were provided, pass them directly to CLI
if not "%~1"=="" (
    ".venv\Scripts\python.exe" -m conveyor_control.cli %*
    exit /b %ERRORLEVEL%
)

cls
echo =========================================================================
echo   P15 - CONVEYOR MOTOR SPEED CONTROL // BANG-BANG, PWM AND PID
echo =========================================================================
echo.
echo   [1] Launch SCADA Web Control Room Dashboard (Recommended)
echo   [2] Launch Rich Terminal Command-Line Dashboard
echo   [3] Launch PyBullet 3D Digital Twin Visualizer
echo   [4] Run Full Comparative Experiment Suite (--compare)
echo   [5] Run All Automated Tests (pytest)
echo   [6] Exit
echo.
echo =========================================================================
set /p CHOICE="Select an option [1-6, default 1]: "

if "%CHOICE%"=="" set CHOICE=1
if "%CHOICE%"=="1" (
    ".venv\Scripts\python.exe" run_dashboard.py
    exit /b 0
)
if "%CHOICE%"=="2" (
    ".venv\Scripts\python.exe" -m conveyor_control.cli
    exit /b 0
)
if "%CHOICE%"=="3" (
    ".venv\Scripts\python.exe" -m conveyor_control.cli --controller pid --scenario load_cycle --visualize
    exit /b 0
)
if "%CHOICE%"=="4" (
    ".venv\Scripts\python.exe" -m conveyor_control.cli --compare
    pause
    exit /b 0
)
if "%CHOICE%"=="5" (
    ".venv\Scripts\python.exe" -m pytest tests/ -v
    pause
    exit /b 0
)
if "%CHOICE%"=="6" (
    exit /b 0
)

echo Invalid selection.
pause
exit /b 1

:setup_failed
echo Setup failed. See the error above and the Installation section in README.md.
pause
exit /b 1

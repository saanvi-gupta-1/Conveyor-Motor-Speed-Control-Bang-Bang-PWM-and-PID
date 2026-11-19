@echo off
setlocal
cd /d "%~dp0"

echo =========================================================================
echo   STARTING CONVEYOR MOTOR SPEED CONTROL // SCADA CONTROL ROOM
echo =========================================================================
echo.

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" run_dashboard.py
) else (
    python run_dashboard.py
)

pause

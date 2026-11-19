@echo off
title Conveyor Motor Speed Control: Bang-Bang, PWM & PID (P15)
color 0B
echo ===============================================================================
echo          CONVEYOR MOTOR SPEED CONTROL: BANG-BANG, PWM & PID (P15)
echo ===============================================================================
echo.
echo Launching PyBullet 3D Simulation and Interactive Dashboard...
echo.

cd /d "%~dp0"
"C:\Users\Lenovo\AppData\Local\Python\bin\python.exe" main.py

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [!] Python returned exit code %ERRORLEVEL%. Attempting default python...
    python main.py
)

echo.
echo Simulation finished.
pause

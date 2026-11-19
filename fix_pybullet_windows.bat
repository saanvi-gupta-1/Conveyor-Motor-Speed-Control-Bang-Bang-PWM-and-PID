@echo off
setlocal
cd /d "%~dp0"

echo =========================================================================
echo       PYBULLET WINDOWS SMART APP CONTROL (SAC) UNBLOCK HELPER
echo =========================================================================
echo.
echo ROOT CAUSE IDENTIFIED:
echo   Windows 11 Smart App Control (SAC) is currently set to "ON".
echo   SAC blocks unsigned C/C++ Python native extensions (.pyd/.dll).
echo   Event Log confirmation: CodeIntegrity Event 3118 (Smart App Control Block).
echo.
echo THE FIX (Takes 15 seconds):
echo   1. Windows Security will open automatically in a moment.
echo   2. Click "App & browser control".
echo   3. Click "Smart App Control settings".
echo   4. Select "Off" (or "Evaluation" mode).
echo   5. Click "Yes" on the Windows Administrator prompt.
echo.
echo =========================================================================
echo Opening Windows Security settings now...
echo =========================================================================

start windowsdefender://appbrowser

echo.
echo Once you have toggled Smart App Control to "Off" in Windows Security,
pause
echo.
echo Verifying PyBullet now...
echo.

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" verify_pybullet.py
) else (
    python verify_pybullet.py
)

echo.
pause

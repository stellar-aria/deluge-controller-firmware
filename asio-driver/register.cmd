@echo off
:: Register DelugeASIO.dll as a COM/ASIO driver
:: Must be run as Administrator

net session >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Run this script as Administrator.
    pause
    exit /b 1
)

set "DLL=%~dp0build\bin\Release\DelugeASIO.dll"
if not exist "%DLL%" (
    echo ERROR: %DLL% not found. Build the Release config first.
    pause
    exit /b 1
)

regsvr32 "%DLL%"

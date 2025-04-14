@echo off
setlocal enabledelayedexpansion

rem Get the directory of the script
set SCRIPT_DIR=%~dp0

echo ==========================
echo Arduino Software Installer
echo ==========================
echo.

echo Current working directory:
echo %SCRIPT_DIR%
echo. 

set "ports="
:: Get a list of COM ports and store in a variable
for /f "tokens=2 delims==" %%a in ('wmic path Win32_SerialPort get DeviceID /format:list 2^>nul ^| find "DeviceID"') do (
    if "%%a" neq "No Instance(s) Available." (
        set "ports=!ports! %%a"
    )
)

if "%ports%" == "" (
    echo COM Ports:
    echo No COM ports found.
) else (
    :: Remove duplicates
    set "unique_ports="
    for %%a in (%ports%) do (
        if "!unique_ports!" == "" (
            set "unique_ports=%%a"
        ) else (
            if "!unique_ports!" neq "!unique_ports:%%a=!" (
                set "unique_ports=!unique_ports! %%a"
            )
        )
    )

    :: Display the unique COM ports
    echo COM Ports:
    for %%a in (!unique_ports!) do echo %%a
)
echo.

rem Prompt the user for the COM port
set /p COM_PORT=Enter the COM port (e.g., COM1):

echo You entered COM port: %COM_PORT%

rem Check if COM port exists
mode %COM_PORT% 
echo Using %COM_PORT%
cd %SCRIPT_DIR%
bossac.exe -i -d --port=%COM_PORT% -U -i --offset=0x2000 -w -v FS_FAL_M0.ino.bin -R
pause
endlocal



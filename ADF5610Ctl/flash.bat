@echo off
cd /D "%~dp0"

:start
set /p port="Enter port where device is connected: "
echo Flashing using port %port%

avrdude.exe -Cavrdude.conf -v -patmega328p -carduino -P%port% -b57600 -D -Uflash:w:ADF5610Ctl.ino.eightanaloginputs.hex:i

if ERRORLEVEL 1 goto fail

exit /b 0

:fail
choice /M "Flashing failed, do you want to retry"
if %ERRORLEVEL% EQU 1 goto start

exit /b 1

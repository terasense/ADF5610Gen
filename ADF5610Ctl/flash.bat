@echo off
setlocal
cd /D "%~dp0"
set fw_path="ADF5610Ctl.ino.promicro.hex"

:start
echo Available ports:
for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "USB Serial"') do (
    echo %%I
    set port=%%I
)
for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "Pro Micro"') do (
    echo %%I
    set port=%%I
)
echo.
if not defined port (
	set /p port="Enter port where device is connected: "
) else (
	set /p port="Enter port where device is connected or press Enter to use %port%: "
)

echo Resetting port %port%
mode %port%: BAUD=1200 parity=N data=8 stop=1

echo Auto detecting programming port
for /L %%I in (1,1,10) do (
	timeout /nobreak /t 1
	for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "Pro Micro"') do (
	    set pport=%%I
	    goto flash
	)
	for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "USB bootloader"') do (
	    set pport=%%I
	    goto flash
	)
	for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "USB Serial"') do (
	    set pport=%%I
	    goto flash
	)
)

echo Can't find programming port
goto error

:flash
echo Programming using port %pport%
avrdude.exe -Cavrdude.conf -v -patmega32u4 -cavr109 -P%pport% -b57600 -D -Uflash:w:%fw_path%:i

if NOT ERRORLEVEL 1 goto done

echo Programming failed

:error
echo Available ports:
for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "USB Serial"') do (
    echo %%I
)
for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "USB bootloader"') do (
    echo %%I
)
for /f "tokens=2 delims=()" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "Pro Micro"') do (
    echo %%I
)
echo.
choice /M "Do you want to retry"
if %ERRORLEVEL% EQU 1 goto start
exit /b 1

:done
exit /b 0

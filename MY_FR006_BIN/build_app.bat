@echo off
call C:\ti\scripts\tisdklk 4.3.0.1
call C:\ti\mmwave_mcuplus_sdk_04_03_00_01\mmwave_mcuplus_sdk_04_03_00_01\scripts\windows\setenv.bat awr2944 A0_FR

set AY_BAT=%cd%
set APP_PATH=%cd%

cd %APP_PATH%
touch %APP_PATH%\mss\mmw_cli.c
set MAKE=gmake
%MAKE% clean
@REM %MAKE% all radar_type=A7_CR sparse_array=X02 autosar_package=enable mcan_spi_enable=enable autosar_wake_prot=disable pti_cfar=enable on_performance=enable flash_type=b32q meta_image=local
%MAKE% all radar_type=A7_FR sparse_array=X02 autosar_package=enable mcan_spi_enable=enable autosar_wake_prot=disable pti_cfar=enable range768=enable on_performance=enable flash_type=b32q meta_image=local 
@REM copy awr2944_a7_front_radar.appimage C:\work\01_jobs\01_MoisUDS27svc\98_tools\18_Chery_serial\chery_app.appimage

copy awr2944_a7_front_radar.appimage C:\Users\zhujunnan\Desktop\FR006_zlq_awr2944_V23\MY_FR006_BIN

cd C:\Users\zhujunnan\Desktop\FR006_zlq_awr2944_V23\MY_FR006_BIN

setlocal enabledelayedexpansion
for /f "tokens=1-3 delims=:. " %%a in ('echo %time%') do (
    set HH=%%a
    set MM=%%b
    set SS=%%c
)

for /f "tokens=2-4 delims=/ " %%a in ('echo %date%') do (
    set YYYY=%%c
    set MM=%%a
    set DD=%%b
)

@REM set HH=%HH: =0%
@REM set MM=%MM: =0%

set NewFileName=FR006_%MM%_%DD%_%HH%_%MM%_%ss%_CALIBRATION.bin
ren awr2944_a7_front_radar.appimage %NewFileName%

cd %AY_BAT%
@echo off

call "setup_mingw.bat"

cd .

if "%1"=="" ("C:\PROGRA~1\MATLAB\R2022a\bin\win64\gmake"  -f disturbanceObserver_rigid_model_rtw.mk all) else ("C:\PROGRA~1\MATLAB\R2022a\bin\win64\gmake"  -f disturbanceObserver_rigid_model_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1
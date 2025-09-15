

call  "\\EBE-GREEN-019\C$\Program Files\MATLAB\R2025a\bin\win64\checkMATLABRootForDriveMap.exe" "\\EBE-GREEN-019\C$\Program Files\MATLAB\R2025a"  > mlEnv.txt
for /f %%a in (mlEnv.txt) do set "%%a"\n
cd .

if "%1"=="" ("C:\Program Files\MATLAB\R2025a\bin\win64\gmake" MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  -f MicroMouse_Deploy.mk all) else ("C:\Program Files\MATLAB\R2025a\bin\win64\gmake" MATLAB_ROOT=%MATLAB_ROOT% ALT_MATLAB_ROOT=%ALT_MATLAB_ROOT% MATLAB_BIN=%MATLAB_BIN% ALT_MATLAB_BIN=%ALT_MATLAB_BIN%  -f MicroMouse_Deploy.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1
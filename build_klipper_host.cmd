@echo off
setlocal
set "VCTargetsPath=C:\Program Files\Microsoft Visual Studio\18\Insiders\MSBuild\Microsoft\VC\v180\"
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 exit /b %errorlevel%
cd /d "C:\GitHub\klipper_host_cpp"
msbuild .\klipper_host\klipper_host.vcxproj /p:Configuration=Debug /p:Platform=x64 /m
exit /b %errorlevel%
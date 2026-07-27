@echo off
setlocal
set "ROOT=%~dp0"
cd /d "%ROOT%"

call "%ROOT%setup.bat"
if errorlevel 1 exit /b %errorlevel%

where msbuild >nul 2>nul
if errorlevel 1 (
    echo MSBuild not found in PATH. Please run this from a Visual Studio Developer Prompt.
    exit /b 1
)

msbuild "%ROOT%klipper_gui\klipper_gui.vcxproj" /p:Configuration=Debug /p:Platform=x64 /m
if errorlevel 1 exit /b %errorlevel%

if exist "%ROOT%x64\Debug\klipper_gui.exe" (
    start "" "%ROOT%x64\Debug\klipper_gui.exe"
) else (
    echo GUI executable was not found in x64\Debug.
    exit /b 1
)

endlocal

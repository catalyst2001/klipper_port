@echo off
setlocal

echo ============================================
echo  klipper_host_cpp - Setup Dependencies
echo ============================================
echo.

set "ROOT=%~dp0"
set "VCPKG_DIR=%ROOT%vcpkg"
set "VCPKG_EXE=%VCPKG_DIR%\vcpkg.exe"
set "TRIPLET=x64-windows"

:: ---- Step 1: Clone vcpkg if missing ----
if not exist "%VCPKG_DIR%\.vcpkg-root" (
    echo [1/3] Cloning vcpkg...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_DIR%"
    if errorlevel 1 (
        echo ERROR: Failed to clone vcpkg. Make sure git is installed and in PATH.
        exit /b 1
    )
) else (
    echo [1/3] vcpkg directory already exists, skipping clone.
)

:: ---- Step 2: Bootstrap vcpkg if exe missing ----
if not exist "%VCPKG_EXE%" (
    echo [2/3] Bootstrapping vcpkg...
    call "%VCPKG_DIR%\bootstrap-vcpkg.bat" -disableMetrics
    if errorlevel 1 (
        echo ERROR: vcpkg bootstrap failed.
        exit /b 1
    )
) else (
    echo [2/3] vcpkg.exe already exists, skipping bootstrap.
)

:: ---- Step 3: Install packages ----
echo [3/3] Installing packages for %TRIPLET%...
echo.

:: wxWidgets (GUI framework) - pulls in all transitive deps (zlib, libpng, etc.)
echo Installing wxwidgets...
"%VCPKG_EXE%" install wxwidgets:%TRIPLET%
if errorlevel 1 (
    echo ERROR: Failed to install wxwidgets.
    exit /b 1
)

:: ---- Step 4: Copy wxWidgets runtime DLLs into output folders ----
echo.
echo Copying wxWidgets runtime DLLs to build output folders...
for %%C in (Debug Release) do (
    if exist "%ROOT%x64\%%C\" (
        if /I "%%C"=="Debug" (
            xcopy /Y /D "%VCPKG_DIR%\installed\%TRIPLET%\debug\bin\*.dll" "%ROOT%x64\%%C\" >nul
        ) else (
            xcopy /Y /D "%VCPKG_DIR%\installed\%TRIPLET%\bin\*.dll" "%ROOT%x64\%%C\" >nul
        )
    )
)

echo.
echo ============================================
echo  Setup complete!
echo.
echo  You can now open klipper_host_cpp.slnx
echo  and build with Configuration=Debug,
echo  Platform=x64.
echo  The GUI executable will now find its wxWidgets DLLs automatically.
echo ============================================

endlocal

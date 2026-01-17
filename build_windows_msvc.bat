@echo off
REM ===========================================
REM FAST-LIO Windows Build Script (MSVC)
REM ===========================================

echo.
echo ========================================
echo   FAST-LIO Windows Build (MSVC)
echo ========================================
echo.

REM Check for Visual Studio environment
if "%VSINSTALLDIR%"=="" (
    echo [INFO] Visual Studio environment not detected.
    echo [INFO] Trying to find and setup VS2022...
    
    REM Try VS2022 at D:\Visual_Studio2022
    if exist "D:\Visual_Studio2022\VC\Auxiliary\Build\vcvars64.bat" (
        call "D:\Visual_Studio2022\VC\Auxiliary\Build\vcvars64.bat"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
        call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
    ) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" (
        call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
    ) else (
        echo [ERROR] Could not find Visual Studio 2022.
        echo [ERROR] Please run this script from Developer Command Prompt.
        pause
        exit /b 1
    )
)

echo.
echo [INFO] Using compiler: cl.exe
echo.

REM Set paths (strip trailing backslash from %~dp0 to avoid escaping quote)
set PROJECT_DIR=%~dp0
set PROJECT_DIR=%PROJECT_DIR:~0,-1%
set BUILD_DIR=%PROJECT_DIR%\build_msvc

REM Create build directory
if not exist "%BUILD_DIR%" (
    echo [INFO] Creating build directory...
    mkdir "%BUILD_DIR%"
)

cd /d "%BUILD_DIR%"

REM Run CMake with NMake
echo [INFO] Configuring CMake...
cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release "%PROJECT_DIR%"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] CMake configuration failed!
    pause
    exit /b 1
)

REM Build
echo.
echo [INFO] Building project...
nmake

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Build failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo   Build completed successfully!
echo ========================================
echo.
echo Output files are in: %BUILD_DIR%
echo   - fastlio_mapping.dll
echo   - fastlio_mapping.lib
echo.

pause

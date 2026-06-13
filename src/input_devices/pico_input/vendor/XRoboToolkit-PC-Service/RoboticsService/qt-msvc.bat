@echo off
setlocal enabledelayedexpansion

REM Set paths for Qt and MSVC
set QT_DIR=C:\Qt\6.6.3\msvc2019_64
set QT_TOOLS_DIR=C:\Qt\Tools\CMake_64\bin
set NINJA_PATH=C:\Qt\Tools\Ninja
set MSVC_DIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\bin\Hostx64\x64
set VS_DIR=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community

REM Setup Visual Studio environment
call "%VS_DIR%\VC\Auxiliary\Build\vcvars64.bat"

set BASE_DIR=%CD%
set SOURCE_DIR=%BASE_DIR%
set BUILD_DIR=%BASE_DIR%\..\build-RoboticsService-Desktop_Qt_6_6_3_MSVC2019_64bit-RelWithDebInfo
set BUILD_TYPE=RelWithDebInfo

REM Create build directory if it doesn't exist
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM Create .qtc/package-manager directory
if not exist "%BUILD_DIR%\.qtc\package-manager" mkdir "%BUILD_DIR%\.qtc\package-manager"

REM Create empty auto-setup.cmake file if it doesn't exist
if not exist "%BUILD_DIR%\.qtc\package-manager\auto-setup.cmake" (
    echo # Auto-generated file > "%BUILD_DIR%\.qtc\package-manager\auto-setup.cmake"
)

echo.
echo ===== Configuring CMake for RoboticsService =====
echo.

REM Add Ninja to PATH
set PATH=%NINJA_PATH%;%PATH%

REM Run CMake configuration
"%QT_TOOLS_DIR%\cmake.exe" -S "%SOURCE_DIR%" -B "%BUILD_DIR%" -DCMAKE_GENERATOR:STRING=Ninja -DCMAKE_MAKE_PROGRAM="%NINJA_PATH%\ninja.exe" -DCMAKE_BUILD_TYPE:STRING=%BUILD_TYPE% -DCMAKE_PROJECT_INCLUDE_BEFORE:FILEPATH=%BUILD_DIR%/.qtc/package-manager/auto-setup.cmake -DQT_QMAKE_EXECUTABLE:FILEPATH="%QT_DIR%\bin\qmake.exe" -DCMAKE_PREFIX_PATH:PATH="%QT_DIR%" -DCMAKE_C_COMPILER:FILEPATH="%MSVC_DIR%\cl.exe" -DCMAKE_CXX_COMPILER:FILEPATH="%MSVC_DIR%\cl.exe" -DCMAKE_CXX_FLAGS_INIT:STRING= -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_POLICY_DEFAULT_CMP0091=NEW -DCMAKE_POLICY_DEFAULT_CMP0111=NEW

if %ERRORLEVEL% neq 0 (
    echo.
    echo CMake configuration failed with error code %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)

echo.
echo ===== Building RoboticsService =====
echo.

REM Build the project
"%QT_TOOLS_DIR%\cmake.exe" --build "%BUILD_DIR%" --target all

if %ERRORLEVEL% neq 0 (
    echo.
    echo Build failed with error code %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)

echo.
echo ===== Build completed successfully =====
echo.

endlocal

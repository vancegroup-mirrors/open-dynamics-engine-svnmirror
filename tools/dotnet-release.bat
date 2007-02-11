@echo off
rem ***********************************************************
rem * Ode.NET Release Script
rem * Originally written by Jason Perkins (starkos@gmail.com)
rem *
rem * Prerequisites:
rem *  Command-line svn installed on path
rem *  Command-line zip installed on path
rem *  Run within Visual Studio 2005 command prompt
rem ***********************************************************

rem * Check arguments
if "%1"=="" goto show_usage


rem ***********************************************************
rem * Pre-build checklist
rem ***********************************************************

echo. 
echo STARTING PREBUILD CHECKLIST, PRESS ^^C TO ABORT.
echo.
echo Is the version number "%1" correct?
pause
echo.
echo Have you updated the version in AssemblyInfo.cs?
pause
echo.
echo Are 'svn' and 'zip' on the path?
pause
echo.
echo Okay, ready to build the .NET package for version %1!
pause


rem ***********************************************************
rem * Retrieve source code
rem ***********************************************************

echo.
echo RETRIEVING SOURCE CODE FROM REPOSITORY...
echo.

svn export https://svn.sourceforge.net/svnroot/opende/branches/%1/contrib/Ode.NET Ode.NET-%1


rem ***********************************************************
rem * Prepare source code
rem ***********************************************************

echo.
echo PREPARING SOURCE CODE FROM REPOSITORY...
echo.

cd Ode.NET-%1
premake --clean --target vs2005


rem ***********************************************************
rem * Build the binaries
rem ***********************************************************

echo.
echo BUILDING BINARIES...
echo.

devenv.exe Ode.NET.sln /build Debug
devenv.exe Ode.NET.sln /build Release


rem ***********************************************************
rem * Package things up
rem ***********************************************************

rmdir /s /q Ode\obj
erase Ode\Ode.NET.csproj.user
erase Ode.NET.suo

cd ..
zip -r9 Ode.NET-%1.zip Ode.NET-%1\*


rem ***********************************************************
rem * Clean up
rem ***********************************************************

echo.
echo CLEANING UP...
echo.

rmdir /s /q Ode.NET-%1


rem ***********************************************************
rem * Upload to SF.net
rem ***********************************************************

echo.
echo Ready to upload package to SourceForce, press ^^C to abort.
pause
ftp -s:ftp_dotnet_script upload.sourceforge.net
goto done


rem ***********************************************************
rem * Error messages
rem ***********************************************************

:show_usage
echo Usage: dotnet_release.bat version_number
goto done

:done



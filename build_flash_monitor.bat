@echo off
setlocal

set "IDF_ACTION=%~1"
if /I "%IDF_ACTION%"=="--version" goto version
if /I "%IDF_ACTION%"=="build" goto selected_action
if /I "%IDF_ACTION%"=="flash" goto selected_action
if /I "%IDF_ACTION%"=="monitor" goto selected_action
if /I "%IDF_ACTION%"=="clean" goto selected_action
if /I "%IDF_ACTION%"=="all" goto selected_action
set "IDF_ACTION=all"
set "IDF_ARGS=%*"
goto invoke

:selected_action
set "IDF_ARGS="
shift
:collect_args
if "%~1"=="" goto invoke
set "IDF_ARGS=%IDF_ARGS% %1"
shift
goto collect_args

:version
set "IDF_ACTION=version"
set "IDF_ARGS="

:invoke
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "& { . 'C:\Espressif\tools\Microsoft.v6.0.PowerShell_profile.ps1'; Set-Location -LiteralPath '%~dp0'; switch ('%IDF_ACTION%') { 'build' { idf.py %IDF_ARGS% build } 'flash' { idf.py %IDF_ARGS% flash } 'monitor' { idf.py %IDF_ARGS% monitor } 'clean' { idf.py %IDF_ARGS% clean } 'version' { idf.py --version } default { idf.py %IDF_ARGS% build flash monitor } }; exit $LASTEXITCODE }"
exit /b %ERRORLEVEL%
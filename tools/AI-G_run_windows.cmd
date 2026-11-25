@echo off
powershell.exe -NoProfile -ExecutionPolicy RemoteSigned -File "%~dp0AI-G_run_windows.ps1" %*
exit /b %errorlevel%

@echo off
SET "VENV_PATH=C:\Users\florianmaurer\Documents\GitHub\PhotographyRig\env\Scripts\Activate.ps1"
SET "PYTHON_DIR=C:\Users\florianmaurer\AppData\Local\Programs\Python\Python39"

echo Activating virtual environment...
PowerShell -NoExit -Command "Start-Process PowerShell -ArgumentList '-NoProfile -ExecutionPolicy Bypass -NoExit -Command \". ''%VENV_PATH%''; cd ''%~dp0''; $env:Path = ''%PYTHON_DIR%'' + '';$env:Path'';\"' -Verb RunAs"

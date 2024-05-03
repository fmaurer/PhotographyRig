@echo off
SET "PYTHON_DIR=C:\Users\florianmaurer\AppData\Local\Programs\Python\Python39"
SET "VENV_PATH=env"

if not exist "%VENV_PATH%" (
    echo Creating virtual environment...
    "%PYTHON_DIR%\python.exe" -m venv "%VENV_PATH%"
    echo Virtual environment created.
)

echo Activating virtual environment...
call "%VENV_PATH%\Scripts\activate"

echo Installing dependencies...
pip install -r requirements.txt

echo Setup complete. Virtual environment is ready to use.
pause
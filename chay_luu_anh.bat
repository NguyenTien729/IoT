@echo off
chcp 65001 >nul
cd /d "%~dp0"
py -3 anh_an_ninh_server.py
if errorlevel 1 (
  echo Khong khoi dong duoc bang py. Thu python...
  python anh_an_ninh_server.py
)
pause

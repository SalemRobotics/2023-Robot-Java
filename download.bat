@echo off
sftp lvuser@10.63.24.2:/home/lvuser/tuning.txt C:\Users\sau57\Desktop\
if %ERRORLEVEL% equ 0 echo File Cloned. || echo %ERROR%
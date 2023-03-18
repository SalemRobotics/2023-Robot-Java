@echo off
sftp lvuser@10.63.24.2:/home/lvuser/tuning.csv C:\Users\sau57\Desktop\
ssh lvuser@10.63.24.2 rm /home/lvuser/tuning.csv
if %ERRORLEVEL% equ 0 echo File Cloned. || echo %ERROR%
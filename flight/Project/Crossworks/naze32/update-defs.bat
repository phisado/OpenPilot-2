@echo off
set MKFILE=..\..\..\..\make\boards\naze32\board-info.mk
set OUTPUT1=board-info.def
set OUTPUT2=linker.def

del %OUTPUT1%
del %OUTPUT2%

for /F "tokens=1,3" %%A in (%MKFILE%) do (
  for /f %%i in ('echo %%A ^| find "FW_"') do  (
    echo #define %%A %%B>> %OUTPUT1%
    echo --defsym=%%A=%%B>> %OUTPUT2%
  )
  for /f %%i in ('echo %%A ^| find "EE_"') do  (
    echo #define %%A %%B>> %OUTPUT1% 
    echo --defsym=%%A=%%B>> %OUTPUT2%
  )
)

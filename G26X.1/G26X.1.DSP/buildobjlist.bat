@echo off
setlocal enabledelayedexpansion

set LIST_CPP=modules_cpp = 
set LIST_OBJ=modules_obj = 
set LIST_DIR=cpp_dir_list = %1
set LIST_INC=inc_dir_list = %1

@rem echo %1 %2

for /D /R %1 %%i in ("*") do (
	set TEMP=%%i
	set LIST_DIR=!LIST_DIR!;!TEMP!
	set LIST_INC=!LIST_INC!,!TEMP!
	set TEMP=!TEMP:%1=%2!
	rem set LIST_OBJ_DIR=!LIST_OBJ_DIR!;!TEMP!
	if NOT EXIST !TEMP! md !TEMP!
	rem echo %%i
	rem echo !TEMP!
)
echo !LIST_DIR! > %2\cppdirlist
echo !LIST_INC! > %2\incdirlist
 
if EXIST %2mkoutdep del /Q %2mkoutdep

for /R %1 %%i in ("*.cpp") do (
	set TEMP=%%i
	set LIST_CPP=!LIST_CPP! !TEMP!
	set TEMP=!TEMP:.cpp=.doj!
	set TEMP=!TEMP:%1=%2!
	set DEP=!TEMP:.doj=.d!
	if NOT EXIST !DEP! echo !TEMP! : %%i > !DEP!
	type !DEP! >> %2mkoutdep
	set LIST_OBJ=!LIST_OBJ! !TEMP!
	rem echo %%i
	rem echo !TEMP!
	rem echo !DEP!
)

echo !LIST_CPP! > %2\mkoutcpp
echo !LIST_OBJ! > %2\mkoutobj

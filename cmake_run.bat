:: Generated Makefiles for QtCreator.
:: Project build will be done by visual c++ through QtCreator which is going to
:: run the Makefiles with jom.exe
:: N.B: jom must be accessible from %PATH%


:: Test for script arguments
@IF "%1" == "" (
	@echo SCRIPT USAGE:
	@echo First arg: The cmake build type Debug/Release/...
	@echo Second arg: the generation type -G"NMake Makefiles JOM" or "" for visual
	@echo.
	@echo Your args: "%1" "%2"
	pause
	exit	
)


:: Test environment variables are well defined
@IF "%VISUAL_STUDIO_2010_PATH%" == "" (	
@	echo ============================================================
@	echo ERROR CAN'T FIND PATH TO VISUAL STUDIO
@	echo please define the environment variable:
@	echo VISUAL_STUDIO_2010_PATH=path_to_visual_compiler_binaries
@	echo the path usually looks like: 
@	echo "C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\bin"
@	echo ============================================================
	pause
	exit
) else (	
@	echo "Visual studio path: %VISUAL_STUDIO_2010_PATH%"	
)

@IF "%CUDA_COMPUTE_CAPABILITY%" == "" (	
@	echo ============================================================
@	echo ERROR CAN'T FIND COMPUTE CAPABILITY
@	echo please define the environment variable:
@	echo CUDA_COMPUTE_CAPABILITY=sm_xx
@	echo where sm_xx is the CUDA compute capability of your GPU 
@	echo sm_10 sm_13 etc.
@	echo ============================================================
	pause
	exit
) else (	
@	echo "GPU compute capability: %CUDA_COMPUTE_CAPABILITY%"	
)

@echo. 
@echo.
@echo ========== LAUNCH CMAKE ==========
@echo.
@echo.

:: setup the evironment variables needed by visual studio
call "%VISUAL_STUDIO_2010_PATH%\vcvars32.bat"

cd build
erase CMakeCache.txt

:: NB: We use '/' instead of '\' when telling visual studio path
:: This is done with %ENV_VAR:\=/% which replace every backslashes with slashes
cmake -DCMAKE_BUILD_TYPE=%1 -DCMAKE_RC_COMPILER="%VISUAL_STUDIO_2010_PATH:\=/%" -DCUDA_COMPUTE_CAPABILITY=%CUDA_COMPUTE_CAPABILITY% %2 ./..

cd ..

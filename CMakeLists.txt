# Note set CUDA_BUILD_EMULATION to OFF to enable Device mode compilation.

cmake_minimum_required(VERSION 2.8.11)

project(implicit_framework)

#-------------------------------------------------------------------------------
# Check dependencies
#-------------------------------------------------------------------------------
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

# END CHECK DEPENDANCIES -------------------------------------------------------


# ------------------------------------------------------------------------------
# Setup Cuda
# ------------------------------------------------------------------------------
if(CUDA_FOUND)
   message("CUDA found")
else()
   message("CUDA not found")
endif()

if( EXISTS "$ENV{CUDA_SDK_ROOT_DIR}" )
    message("CUDA SDK path found")
else()
    message("CUDA SDK path not found")
endif()

if( "${CUDA_COMPUTE_CAPABILITY}" STREQUAL "" )
    message(FATAL_ERROR "Please define the symbol CUDA_COMPUTE_CAPABILITY with -D (you can set it to sm_10 sm_13 sm_XX etc.) (cf. your GPU documentation)")
else()
    message("CUDA compute capability defined to ${CUDA_COMPUTE_CAPABILITY}")
endif()
# END SETUP CUDA ---------------------------------------------------------------


#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------

# The include_directories adds paths to both C/C++ compilation in the native
# compiler and cuda compilation in NVCC.  Note that CUDA_INCLUDE_DIRS is added
# automatically by CUDA_ADD_EXECUTABLE and CUDA_ADD_LIBRARY.

# SET(FBX_ROOT $ENV{FBXSDK_ROOT})
SET(FBX_ROOT "C:/Program Files/Autodesk/FBX SDK")

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${FBX_ROOT}/2012.2/include
    libs/include
    libs/include/vcglib
    include
    include/animation
    include/animation/pinocchio
    include/blending_functions
    include/blending_lib
    include/blending_lib/cuda_interface
    include/control
    include/containers
    include/global_datas
    include/maths
    include/maths/intersections
    include/meshes
    include/meshes/voxelizer
    include/meshes/vcg_lib
    include/parsers
    include/primitives/precomputed_prim
    include/primitives/hrbf
    include/primitives
    include/qt_gui
    include/qt_gui/common/gizmo_deprecated
    include/rendering
    include/rendering/environment_map
    include/utils
    include/utils/portable_includes
    include/utils/gl_utils
    include/utils/cuda_utils
    include/scene_tree
    include/implicit_graphs/CSG
    include/implicit_graphs/skinning
  )

file(
    GLOB_RECURSE
    include/qt_gui/SKIN/main_window_skin.hpp
)

# List of nvcc sources
file(
    GLOB_RECURSE
    cuda_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/animation/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/environment_map/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blending_lib/cuda_interface/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gl_utils/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/cuda_utils/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/global_datas/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/precomputed_prim/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/hrbf/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/implicit_graphs/skinning/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu
)

# List of cpu sources
file(
    GLOB_RECURSE
    host_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/animation/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/animation/pinocchio/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/maths/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/implicit_graphs/CSG/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/implicit_graphs/skinning/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/gl_utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/cuda_utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/scene_tree/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qt_gui/SKIN/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/hrbf/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parsers/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blending_lib/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/global_datas/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/meshes/vcg_lib/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/meshes/*.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

set(host_sources ${host_sources} ${source_voxelizer} )

# END PROJECT FILES ------------------------------------------------------------


#-------------------------------------------------------------------------------
# CUDA FLAGS
#-------------------------------------------------------------------------------

# Set CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE when you want to add the same .cu
# file to multiple targets.
set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)

#set(PTAX_VERBOSE --ptxas-options=-v) ## print usefull informations for cuda kernels at compilation

# For GeForce 435m, GT 430, GT 440, GTS 450, GTX 460, GTX 550 Ti, GTX 560, GTX 560 Ti, 500M, Quadro 600, 2000, 4000, 5000, 6000
#set(GPU_ARCH --gpu-architecture=sm_21)
# For GeForce GTX 480, (GF100) GTX 465, GTX 470, Tesla C2050, C2070, S/M2050/70, Quadro Plex 7000, GeForce (GF110) GTX570, GTX580, GTX590
#set(GPU_ARCH --gpu-architecture=sm_20)
# For GTX 280, GeForce GTX 260, GTX 275, GTX 285, GTX 295, Tesla C/M1060, S1070, Quadro CX, FX 3/4/5800
#set(GPU_ARCH --gpu-architecture=sm_13)

#user defined architecture
set(GPU_ARCH --gpu-architecture=${CUDA_COMPUTE_CAPABILITY})

# this automatically activate OLIMIT option for this gpu list :
set(GPU_LIST sm_10 sm_11 sm_12 sm_13)
foreach(val ${GPU_LIST})
    if( "${CUDA_COMPUTE_CAPABILITY}" STREQUAL "${val}")
        set(OLIMIT --opencc-options -OPT:Olimit=999999) ## needed for compute < sm_13
        message("CUDA flag: Setting a higher OLIMIT")
    endif()
endforeach(val)

#set(NVCC_DEBUG_FLAG -G0) ## Enable/disable kernels debugging

# Anything other than -D or /D is not passed along to nvcc.
# works for any compiler
IF(MSVC)
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(/arch:SSE)
    ADD_DEFINITIONS(/arch:SSE2)
    ADD_DEFINITIONS(/D _CRT_SECURE_NO_WARNINGS)
    #ADD_DEFINITIONS(/arch:SSE3)
    #ADD_DEFINITIONS(/arch:MMX)
    set(CUDA_NVCC_FLAGS ${NVCC_DEBUG_FLAG} ${OLIMIT} ${PTAX_VERBOSE} ${GPU_ARCH})
ELSE()
    set(CMAKE_CXX_FLAGS -Wall)
    set(COMPILE_BINDIR --compiler-bindir /usr/bin/gcc-4.6)
    set(CUDA_NVCC_FLAGS ${OLIMIT} ${PTAX_VERBOSE} ${COMPILE_BINDIR} ${GPU_ARCH} --compiler-options=-Wall)
ENDIF(MSVC)

# END CUDA FLAGS ---------------------------------------------------------------

#-------------------------------------------------------------------------------
# Define librairies
#-------------------------------------------------------------------------------

if(MSVC)
    INCLUDE_DIRECTORIES("${CUDA_SDK_ROOT_DIR}/common/inc")
    set(glu "glu32")
    set(OpenGL "opengl32")
    ADD_DEFINITIONS(-DFBXSDK_SHARED) # macro needed to tell fbx we're using dlls
    set(FBX "${FBX_ROOT}/2012.2/lib/vs2010/x86/fbxsdk-2012.2.lib")
else()
    set(LIB_CUDA -lcuda)
    set(glu GLU)
    set(OpenGL GL)
    set(FBX "${CMAKE_CURRENT_SOURCE_DIR}/libs/bin/gcc/x64/libfbxsdk-2012.2-static.a")
    #set(TAUCS_LIBRARIES ${taucs})
endif()

# END DEFINE LIBRARIES ---------------------------------------------------------

#-------------------------------------------------------------------------------
# Copy dlls to build directory
#-------------------------------------------------------------------------------

if(MSVC)
configure_file(${FBX_ROOT}/2012.2/lib/vs2010/x86/fbxsdk-2012.2.dll ${CMAKE_CURRENT_BINARY_DIR}/fbxsdk-2012.2.dll COPYONLY)
endif()

# END COPY DLLS ----------------------------------------------------------------


#-------------------------------------------------------------------------------
# Build static libraries
#-------------------------------------------------------------------------------

# Work around CMake: the same CUDA helpers exist upstream, and it's a different
# version that never actually links cuda.lib.
set(CUDA_LIBRARIES ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY})

# build all .cu files into a static lib cuda_lib
CUDA_ADD_LIBRARY( cuda_lib STATIC  ${cuda_sources} )

# Build all .cpp and qt codes into a static lib implicit_framework
ADD_LIBRARY( implicit_framework STATIC ${host_sources} )
# Libraries implicit_framework needs to be linked against:
TARGET_LINK_LIBRARIES(implicit_framework
    ${LIB_CUDA} cuda_lib
    ${OpenGL} ${glu} ${FBX} -ldl -lpthread)

TARGET_LINK_LIBRARIES(cuda_lib implicit_framework)

# END BUILD LIBRARIES ----------------------------------------------------------

#-------------------------------------------------------------------------------
# Build executable
#-------------------------------------------------------------------------------

# Set skinning executable.
ADD_EXECUTABLE(implicit_skinning
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_skinning.cpp )
# Specify the dependency.
TARGET_LINK_LIBRARIES(implicit_skinning implicit_framework cuda_lib )


# Generating PTX files.
# CUDA_COMPILE_PTX(PTX_FILES CUDA_FILES test_bin.cu)

# Add a special target to clean nvcc generated files.
CUDA_BUILD_CLEAN_TARGET()

# END BUILD EXECUTABLES --------------------------------------------------------

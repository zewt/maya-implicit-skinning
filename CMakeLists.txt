cmake_minimum_required(VERSION 2.8.11)
project(implicit_skinning_maya)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 OLD)
endif()
#-------------------------------------------------------------------------------
# Check dependencies
#-------------------------------------------------------------------------------
FIND_PACKAGE(CUDA REQUIRED)

# END CHECK DEPENDANCIES -------------------------------------------------------

if(CMAKE_BUILD_ARCH_X64)
    message("build set for x64 arch")
endif()

# ------------------------------------------------------------------------------
# Setup Cuda
# ------------------------------------------------------------------------------

set(MAYA_DIR "C:/Program Files/Autodesk/Maya2015")

if(CUDA_FOUND)
   message("CUDA found")
else()
   message("CUDA not found")
endif()

if( "${CUDA_COMPUTE_CAPABILITY}" STREQUAL "" )
    message("CUDA_COMPUTE_CAPABILITY env variable not found ")
    message("   -> set to default value")
    set(CUDA_COMPUTE_CAPABILITY sm_52)
endif()

message("CUDA compute capability defined to ${CUDA_COMPUTE_CAPABILITY}")
# END SETUP CUDA ---------------------------------------------------------------


#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(
    "${MAYA_DIR}/include"
    libs/include
    libs/include/vcglib
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/animation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/animation/pinocchio
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blending_functions
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blending_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blending_lib/cuda_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control
    ${CMAKE_CURRENT_SOURCE_DIR}/src/containers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/global_datas
    ${CMAKE_CURRENT_SOURCE_DIR}/src/maths
    ${CMAKE_CURRENT_SOURCE_DIR}/src/maths/intersections
    ${CMAKE_CURRENT_SOURCE_DIR}/src/meshes
    ${CMAKE_CURRENT_SOURCE_DIR}/src/meshes/voxelizer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/meshes/vcg_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parsers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/precomputed_prim
    ${CMAKE_CURRENT_SOURCE_DIR}/src/primitives/hrbf
    ${CMAKE_CURRENT_SOURCE_DIR}/src/primitives
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qt_gui
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qt_gui/common/gizmo_deprecated
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rendering/environment_map
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/portable_includes
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/gl_utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/cuda_utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/scene_tree
    ${CMAKE_CURRENT_SOURCE_DIR}/src/implicit_graphs
)

# List of nvcc sources
file(GLOB_RECURSE cuda_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu)

# List of cpu sources
file(GLOB_RECURSE host_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# END PROJECT FILES ------------------------------------------------------------


#-------------------------------------------------------------------------------
# CUDA FLAGS
#-------------------------------------------------------------------------------

set( CUDA_SEPARABLE_COMPILATION ON)

#set(PTAX_VERBOSE --ptxas-options=-v) ## print usefull informations for cuda kernels at compilation

# List of GPUs and compatible compute capabilities:
# https://en.wikipedia.org/wiki/CUDA#GPUs_supported

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
if(MSVC)
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(/D _CRT_SECURE_NO_WARNINGS)
    #ADD_DEFINITIONS(/arch:SSE3)
    #ADD_DEFINITIONS(/arch:MMX)
    ADD_DEFINITIONS(/FS)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -rdc=true ${NVCC_DEBUG_FLAG} ${OLIMIT} ${PTAX_VERBOSE} ${GPU_ARCH} "-Xcompiler \"/wd 4819\"")
else()
    set(CMAKE_CXX_FLAGS -Wall)
    set(COMPILE_BINDIR --compiler-bindir /usr/bin/gcc-4.6)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -rdc=true ${OLIMIT} ${PTAX_VERBOSE} ${COMPILE_BINDIR} ${GPU_ARCH} --compiler-options=-Wall)
endif(MSVC)

# END CUDA FLAGS ---------------------------------------------------------------

#-------------------------------------------------------------------------------
# Define librairies
#-------------------------------------------------------------------------------

if(MSVC)
    INCLUDE_DIRECTORIES("${CUDA_SDK_ROOT_DIR}/common/inc")
else()
    set(LIB_CUDA -lcuda)
endif()

# END DEFINE LIBRARIES ---------------------------------------------------------

#-------------------------------------------------------------------------------
# Build static libraries
#-------------------------------------------------------------------------------

# Work around CMake: the same CUDA helpers exist upstream, and it's a different
# version that never actually links cuda.lib.
set(CUDA_LIBRARIES ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY})

# Fix up CMake's broken default project organization.  It wants to put headers
# in a separate "Header Files" group, which isn't usable since you have to jump
# all over the place to find anything.  Put header files next to their corresponding
# source files.  The files themselves need to be rearranged in this way, too.
# Also, group CUDA files together, or it'll put them at the top.

source_group("CUDA" REGULAR_EXPRESSION .*\\.cu$)
source_group("Source" REGULAR_EXPRESSION ".*\\.(cpp|hpp|h)$")

# build all .cu files into a static lib implicit_cuda
CUDA_ADD_LIBRARY( implicit_cuda STATIC  ${cuda_sources} ${host_sources} )

# Libraries implicit_framework needs to be linked against:
TARGET_LINK_LIBRARIES(implicit_cuda ${LIB_CUDA})

if(!MSVC)
    TARGET_LINK_LIBRARIES(implicit_cuda -ldl -lpthread)
endif()

# --------------------------

ADD_LIBRARY(implicit_skinning_maya SHARED src/maya/plugin.cpp)
SET_TARGET_PROPERTIES(implicit_skinning_maya PROPERTIES SUFFIX .mll)
SET_TARGET_PROPERTIES(implicit_skinning_maya PROPERTIES CMAKE_MODULE_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} /DEF:my_defs.def /NODEFAULTLIB")

TARGET_LINK_LIBRARIES(implicit_skinning_maya
    "${MAYA_DIR}/lib/OpenMaya.lib"
    "${MAYA_DIR}/lib/OpenMayaAnim.lib"
    "${MAYA_DIR}/lib/Foundation.lib"
    "${MAYA_DIR}/lib/OpenMayaRender.lib"
    "${MAYA_DIR}/lib/OpenMayaUI.lib"
)

if(MSVC)
    SET_TARGET_PROPERTIES(
        implicit_skinning_maya
        PROPERTIES
        LINK_FLAGS
        "/export:initializePlugin /export:uninitializePlugin")

    ADD_DEFINITIONS(
        -D_USRDLL
        -DNT_PLUGIN
        -D_SECURE_SCL=0
        -D_SECURE_SCL_THROWS=0
        -D_SECURE_SCL_DEPRECATE=0
        -D_CRT_SECURE_NO_DEPRECATE
        -DREQUIRE_IOSTREAM
        -DAW_NEW_IOSTREAMS
        -DBits64_
    )
endif(MSVC)

TARGET_LINK_LIBRARIES(implicit_skinning_maya implicit_cuda)

# END BUILD LIBRARIES ----------------------------------------------------------

# Add a special target to clean nvcc generated files.
CUDA_BUILD_CLEAN_TARGET()

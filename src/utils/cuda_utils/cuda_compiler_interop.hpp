/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#ifndef CUDA_COMPILER_INTEROP_HPP__
#define CUDA_COMPILER_INTEROP_HPP__

#if defined(NO_CUDA)
#if defined(__CUDACC__)
#error NO_CUDA files should not be CUDACC
#endif
#define __host__
#else
#include "cuda_compiler_interop.hpp"
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda.h>
#include <cassert>
#include <stdio.h>
#endif

// -----------------------------------------------------------------------------

/// @def IF_CUDA_DEVICE_HOST
/// @brief set function as device and host when compiled with nvcc
///
/// Only nvcc recognize the keywords __device__ __host__ this macros
/// desactivate them when a file is parsed with other compiler. This
/// way headers can be used with any compiler.
#ifdef __CUDACC__
#define IF_CUDA_DEVICE_HOST __device__ __host__
#else
#define IF_CUDA_DEVICE_HOST
#endif

// -----------------------------------------------------------------------------

/// check if the function is called from device or host side
IF_CUDA_DEVICE_HOST
static inline bool is_device(){
#ifdef __CUDA_ARCH__
    return true;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

/// Checks if current code is host or device code.
/// assert(false) if its host.
/// @note: one could think FORBID_HOST_CALL() is useless.
/// Because nvcc is supposed to raise an error when a function is
/// only __device__ and called from a only __host__ function.
/// Unfortunately when the function is an overloaded operator nvcc does
/// not detect the problem! Worse it tries to execute the code on host
/// side and silently crashes at runtime...
/// Another reason is to avoid compiler errors while preventing someone
/// to use it from host side
#if 0
#ifndef NDEBUG
#define FORBID_HOST_CALL() \
    do{ \
        if( !is_device() ) { \
            printf("ERROR: FUNCTION CALL FORBIDEN FROM HOST CODE\n"); \
            printf("file: %s\n", __FILE__ ); \
            printf("line: %d\n", __LINE__); \
            assert(false); \
         } \
    }while(0)
#else
#define FORBID_HOST_CALL()
#endif
#endif

IF_CUDA_DEVICE_HOST
static inline void FORBID_HOST_CALL(){
    #ifndef __CUDA_ARCH__
        printf("ERROR: FUNCTION CALL FORBIDEN FROM HOST CODE\n");
        assert(false);
    #endif
}

#ifdef NDEBUG
#define FORBID_HOST_CALL()
#endif


// -----------------------------------------------------------------------------


/// Checks if current code is host or device code.
/// assert(false) if its device.
#if 0
#ifndef NDEBUG
#define FORBID_DEVICE_CALL() \
    do{ \
    if( is_device() ) { \
    printf("ERROR: FUNCTION CALL FORBIDEN FROM DEVICE CODE\n"); \
    printf("file: %s\n", __FILE__); \
    assert(false); \
    } \
    }while(0)
#else
#define FORBID_DEVICE_CALL()
#endif
#endif

IF_CUDA_DEVICE_HOST
static inline void FORBID_DEVICE_CALL(){
    #ifdef __CUDA_ARCH__
        printf("ERROR: FUNCTION CALL FORBIDEN FROM DEVICE CODE\n");
        assert(false);
    #endif
}

#ifndef NDEBUG
#define FORBID_DEVICE_CALL()
#endif

#endif // CUDA_COMPILER_INTEROP_HPP__

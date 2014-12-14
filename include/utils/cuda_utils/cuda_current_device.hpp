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
#ifndef CUDA_CURRENT_DEVICE_HPP_
#define CUDA_CURRENT_DEVICE_HPP_

#include <cuda.h>

/**
 * @file cuda_current_device.hpp
 * @brief Some macro utilities with CUDA related to the currently used GPU
 */

/// @return true if the sizes are under the GPU limits
bool check_kernel_size(CUdevice device, const int3 block_size, const int3 grid_size);
bool check_kernel_size(CUdevice device, int block_size, int grid_size);

/// Current device
/// @return current active device (driver identifier)
CUdevice get_cu_device();

#ifndef NDEBUG

#define CUDA_CHECK_KERNEL_SIZE(block, grid) do{\
    if(!check_kernel_size(get_cu_device(), (block), (grid))){\
    fprintf(stderr,"CUDA error: wrong kernel size at %s, line %d\n",\
    __FILE__, __LINE__);\
    fprintf(stderr,"block size: %d grid_size: %d\n", block, grid); \
    fflush(stderr);\
    cuda_print_memory_trace();\
    assert(false);\
    }\
    } while(0)

#else

#define CUDA_CHECK_KERNEL_SIZE(block, grid) do{}while(0)

#endif


/// @brief macro shortcut to launch a CUDA kernel on a linear array
/// with error checking
#define CUDA_LAUNCH_ARRAY(kernel_name, block_size, array_size, ...) \
    do{ \
        const int bl = (block_size); \
        const int gr = (((array_size) + (block_size) - 1) / (block_size)); \
        CUDA_CHECK_KERNEL_SIZE(bl, gr); \
        kernel_name<<<gr, bl>>>(__VA_ARGS__); \
        CUDA_CHECK_ERRORS(); \
    }while(0)


#endif // CUDA_CURRENT_DEVICE_HPP_

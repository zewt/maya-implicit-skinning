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

#endif // CUDA_CURRENT_DEVICE_HPP_

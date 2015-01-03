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
#include "cuda_utils.hpp"

namespace { __device__ void fix_debug() { } }

// =============================================================================
namespace Cuda_utils{
// =============================================================================

int get_max_gflops_device_id()
{
    int current_device   = 0, sm_per_multiproc = 0;
    int max_compute_perf = 0, max_perf_device  = 0;
    int device_count     = 0, best_SM_arch     = 0;
    int arch_cores_sm[3] = { 1, 8, 32 };
    cudaDeviceProp deviceProp;
    cudaGetDeviceCount( &device_count );
    // Find the best major SM Architecture GPU device
    while ( current_device < device_count ) {
        cudaGetDeviceProperties( &deviceProp, current_device );
        if (deviceProp.major > 0 && deviceProp.major < 9999) {
            best_SM_arch = max(best_SM_arch, deviceProp.major);
        }
        current_device++;
    }
    // Find the best CUDA capable GPU device
    current_device = 0;
    while( current_device < device_count ) {
        cudaGetDeviceProperties( &deviceProp, current_device );
        if (deviceProp.major == 9999 && deviceProp.minor == 9999) {
            sm_per_multiproc = 1;
        } else if (deviceProp.major <= 2) {
            sm_per_multiproc = arch_cores_sm[deviceProp.major];
        } else { // Device has SM major > 2
            sm_per_multiproc = arch_cores_sm[2];
        }

        int compute_perf = deviceProp.multiProcessorCount *
                           sm_per_multiproc * deviceProp.clockRate;
        if( compute_perf > max_compute_perf ) {
            // If we find GPU of SM major > 2, search only these
            if ( best_SM_arch > 2 ) {
                // If device==best_SM_arch, choose this, or else pass
                if (deviceProp.major == best_SM_arch) {
                    max_compute_perf  = compute_perf;
                    max_perf_device   = current_device;
                }
            } else {
                max_compute_perf  = compute_perf;
                max_perf_device   = current_device;
            }
        }
        ++current_device;
    }
    cudaGetDeviceProperties(&deviceProp, max_perf_device);
    printf("\nDevice %d: \"%s\"\n", max_perf_device,
           deviceProp.name);
    printf("Compute Capability   : %d.%d\n",
           deviceProp.major, deviceProp.minor);
    return max_perf_device;
}

// -----------------------------------------------------------------------------

void print_device_attribs(CUdevice id)
{
    int res;
    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK,id) );
    printf("Maximum number of threads per block: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_X, id) );
    printf("Maximum x-dimension of a block: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_Y, id) );
    printf("Maximum y-dimension of a block: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_Z,id) );
    printf("Maximum z-dimension of a block: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_X, id) );
    printf("Maximum x-dimension of a grid: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Y, id) );
    printf("Maximum y-dimension of a grid: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Z, id) );
    printf("Maximum z-dimension of a grid: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_BLOCK, id) );
    printf("Maximum amount of shared memory available to a thread block in bytes\n this amount is shared by all thread blocks simultaneously resident on a multiprocessor: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_TOTAL_CONSTANT_MEMORY, id) );
    printf("Memory available on device for __constant__ variables in a CUDA C kernel in bytes: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_WARP_SIZE, id) );
    printf("Warp size in threads: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_PITCH, id) );
    printf("Maximum pitch in bytes allowed by the memory copy functions that involve memory regions allocated through cuMemAllocPitch(): %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_WIDTH, id) );
    printf("Maximum 1D texture width: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_WIDTH, id) );
    printf("Maximum 2D texture width %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_HEIGHT, id) );
    printf("Maximum 2D texture height %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_WIDTH, id) );
    printf("Maximum 3D texture width %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_HEIGHT, id) );
    printf("Maximum 3D texture height %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_DEPTH, id) );
    printf("Maximum 3D texture depth %d\n", res);
#if 0
    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_LAYERED_WIDTH, id) );
    printf("Maximum 1D layered texture width: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_LAYERED_LAYERS, id) );
    printf("Maximum layers in a 1D layered texture: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res,CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LAYERED_WIDTH ,id) );
    printf("Maximum 2D layered texture width: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LAYERED_HEIGHT, id) );
    printf("Maximum 2D layered texture height: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LAYERED_LAYERS, id) );
    printf("Maximum layers in a 2D layered texture %d\n", res);
#endif
    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_REGISTERS_PER_BLOCK, id) );
    printf("Maximum number of 32-bit registers available to a thread block\n this number is shared by all thread blocks simultaneously resident on a multiprocessor: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_CLOCK_RATE, id) );
    printf("Peak clock frequency in kilohertz %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res,CU_DEVICE_ATTRIBUTE_TEXTURE_ALIGNMENT, id) );
    printf("Alignment requirement; texture base addresses aligned to textureAlign bytes do not need an offset applied to texture fetches: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_GPU_OVERLAP, id) );
    printf("1 if the device can concurrently copy memory between host and device while executing a kernel, or 0 if not: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, id) );
    printf("Number of multiprocessors on the device: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_KERNEL_EXEC_TIMEOUT, id) );
    printf("1 if there is a run time limit for kernels executed on the device, or 0 if not: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_INTEGRATED, id) );
    printf("1 if the device is integrated with the memory subsystem, or 0 if not: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_CAN_MAP_HOST_MEMORY, id) );
    printf("1 if the device can map host memory into the CUDA address space, or 0 if not: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_COMPUTE_MODE, id) );
    printf("Compute mode that device is currently in.\n");

    switch(res)
    {
    case CU_COMPUTEMODE_DEFAULT:
        printf("Default mode - Device is not restricted and can have multiple CUDA contexts present at a single time\n");
        break;
    case CU_COMPUTEMODE_EXCLUSIVE:
        printf("Compute-exclusive mode - Device can have only one CUDA context present on it at a time\n");
        break;
    case CU_COMPUTEMODE_PROHIBITED:
        printf("Compute-prohibited mode - Device is prohibited from creating new CUDA contexts\n");
        break;
    }

#if 0
    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_COMPUTEMODE_EXCLUSIVE_PROCESS, id) );
    printf("Compute-exclusive-process mode - Device can have only one context used by a single process at a time: %d\n", res);
#endif
    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_CONCURRENT_KERNELS, id) );
    printf("1 if the device supports executing multiple kernels within the same context simultaneously, or 0 if not.\n It is not guaranteed that multiple kernels will be resident on the device concurrently so this feature should not be relied upon for correctness: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_ECC_ENABLED, id) );
    printf("1 if error correction is enabled on the device, 0 if error correction is disabled or not supported by the device: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_PCI_BUS_ID, id) );
    printf("PCI bus identifier of the device: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_PCI_DEVICE_ID, id) );
    printf("PCI device (also known as slot) identifier of the device: %d\n", res);
#if 0
    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_TCC_DRIVER, id) );
    printf("1 if the device is using a TCC driver. TCC is only available on Tesla hardware running Windows Vista or later: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MEMORY_CLOCK_RATE, id) );
    printf("Peak memory clock frequency in kilohertz: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_GLOBAL_MEMORY_BUS_WIDTH, id) );
    printf("Global memory bus width in bits: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_L2_CACHE_SIZE, id) );
    printf("Size of L2 cache in bytes. 0 if the device doesn't have L2 cache: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_MULTIPROCESSOR, id) );
    printf("Maximum resident threads per multiprocessor: %d\n", res);

    CU_SAFE_CALL(cuDeviceGetAttribute(&res, CU_DEVICE_ATTRIBUTE_UNIFIED_ADDRESSING, id) );
    printf("1 if the device shares a unified address space with the host, or 0 if not: %d\n", res);
#endif
}

// -----------------------------------------------------------------------------

void get_device_memory_usage(double& free, double& total)
{

    size_t free_byte ;
    size_t total_byte ;
    CUDA_SAFE_CALL( cudaMemGetInfo( &free_byte, &total_byte) );

    double free_db  = (double)free_byte;
    double total_db = (double)total_byte;

    free  = (free_db/1024.0/1024.0);
    total = (total_db/1024.0/1024.0);
}

}// END CUDA_UTILS NAMESPACE ===================================================

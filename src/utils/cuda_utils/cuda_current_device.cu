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
#include "cuda_current_device.hpp"

#include "cuda_utils_common.hpp"

// -----------------------------------------------------------------------------

CUdevice get_cu_device()
{
    CUdevice device_id;
    int cuda_device_id;
    cudaGetDevice( &cuda_device_id );
    CU_SAFE_CALL(cuDeviceGet(&device_id, cuda_device_id));
    return device_id;
}

// -----------------------------------------------------------------------------

bool check_kernel_size(CUdevice device_id, const int3 block_size, const int3 grid_size)
{
    int3 max_block_size;
    int3 max_grid_size;
    CU_SAFE_CALL(cuDeviceGetAttribute(&max_block_size.x, CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_X, device_id) );
    CU_SAFE_CALL(cuDeviceGetAttribute(&max_block_size.y, CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_Y, device_id) );
    CU_SAFE_CALL(cuDeviceGetAttribute(&max_block_size.z, CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_Z, device_id) );

    CU_SAFE_CALL(cuDeviceGetAttribute(&max_grid_size.x, CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_X, device_id) );
    CU_SAFE_CALL(cuDeviceGetAttribute(&max_grid_size.y, CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Y, device_id) );
    CU_SAFE_CALL(cuDeviceGetAttribute(&max_grid_size.z, CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Z, device_id) );

    return  (grid_size.x  <= max_grid_size.x ) && (grid_size.y  <= max_grid_size.y ) && (grid_size.z  <= max_grid_size.z) &&
            (block_size.x <= max_block_size.x) && (block_size.y <= max_block_size.y) && (block_size.z <= max_block_size.z);
}

// -----------------------------------------------------------------------------

bool check_kernel_size(CUdevice device, int block_size, int grid_size)
{
    if(block_size < 0 || grid_size < 0) return false;
    return check_kernel_size(device, make_int3(block_size, 0, 0), make_int3(grid_size, 0, 0));
}

// -----------------------------------------------------------------------------

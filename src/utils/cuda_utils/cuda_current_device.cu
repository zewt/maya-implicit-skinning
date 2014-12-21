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

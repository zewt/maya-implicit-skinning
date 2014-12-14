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
#ifndef PRECOMPUTED_PRIM_KERNELS_HPP__
#define PRECOMPUTED_PRIM_KERNELS_HPP__

#include "cuda_utils.hpp"
#include "transfo.hpp"
#include "point_cu.hpp"
#include "skeleton_env_type.hpp"

// =============================================================================
namespace Precomputed_env{
// =============================================================================

/// Filling a 3D grid with an hrbf primitive
/// @param bone_id bone to fill the grid with. Be aware that this id is not
/// the same as bone ids in Skeleton class. use Skeleton_Env::get_idx_device_bone()
/// to convert Skeleton's ids to device ids;
/// @param steps is the (x, y, z) length of each grid cell
/// @param grid_res is the number of cells in the (x, y, z) directions
/// @param org is the 3D coordinates of the origine of the grid.
/// @param transfo the transformation apllied to the implicit primitive which is
/// evaluated
/// @param d_out_grid the linear array to store the grid into.
/// @warning bone_id is the id in device memory ! It is not the same as the one
/// in Skeleton class use Skeleton_Env::get_idx_device_bone()
/// @see Skeleton_Env::get_idx_device_bone()
void fill_grid_with_fngf(Skeleton_env::DBone_id device_bone_id,
                         float3 steps,
                         int grid_res,
                         Point_cu org,
                         Transfo transfo,
                         int grids,
                         int blocks,
                         Cuda_utils::DA_float4 d_out_grid);

}// ============================================================================

#endif //PRECOMPUTED_PRIM_KERNELS_HPP__

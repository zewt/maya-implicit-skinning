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
#include "precomputed_prim_kernels.hpp"

#include "hrbf_env_tex.hpp"

/// These header needs HRBF_Env to be included first
/// @{
#include "hermiteRBF.hpp"
#include "hermiteRBF.inl"
/// @}

#include "blending_env_tex.hpp"

/// This include needs blending_env_tex.hpp but we will not bind it and use it
#include "skeleton_env_tex.hpp"

// -----------------------------------------------------------------------------

/// @warning bone_id in device mem ! not the same as Skeleton class
/// @see Skeleton_Env::get_idx_device_bone()
__global__ static
void fill_grid_kernel(Skeleton_env::DBone_id bone_id,
                      float3 steps,
                      int grid_res,
                      Point_cu org,
                      Transfo transfo,
                      Cuda_utils::DA_float4 d_out_grid)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < d_out_grid.size())
    {
        int x = idx % grid_res;
        int y = (idx / grid_res) % grid_res;
        int z = idx / (grid_res * grid_res);
        Point_cu off = Point_cu(steps.x * x, steps.y * y, steps.z * z);

        Point_cu p = org + off;

        Vec3_cu gf(0.f, 0.f, 0.f);

        HermiteRBF hrbf = Skeleton_env::fetch_bone_hrbf( bone_id );
        float pot = hrbf.fngf(gf, transfo * p);
        pot = pot < 0.00001f ? 0.f  : pot;

        d_out_grid[idx] = make_float4(pot, gf.x, gf.y, gf.z);
    }
}

// =============================================================================
namespace Precomputed_env{
// =============================================================================

void fill_grid_with_fngf(Skeleton_env::DBone_id device_bone_id,
                         float3 steps,
                         int grid_res,
                         Point_cu org,
                         Transfo transfo,
                         int grids,
                         int blocks,
                         Cuda_utils::DA_float4 d_out_grid)
{
    Skeleton_env::bind_local();
    HRBF_env::bind_local();

    fill_grid_kernel<<<grids, blocks >>>
    (device_bone_id, steps, grid_res, org, transfo, d_out_grid);

    Skeleton_env::unbind_local();
    HRBF_env::unbind_local();
}

}// ============================================================================

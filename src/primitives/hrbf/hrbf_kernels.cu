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
#include "hrbf_kernels.hpp"

#include "hrbf_env.hpp"
#include "conversions.hpp"

#include <iostream>

namespace { __device__ void fix_debug() { } }

// =============================================================================
namespace HRBF_kernels{
// =============================================================================

__global__
void hrbf_transform_ker(const int nb_verts,
                        const float4* in_vertices,
                        const float4* in_alpha_beta,
                        const int* map_transfos,
                        const Transfo* transfos,
                        float4* out_vertices,
                        float4* out_alpha_beta)
{
    const int p = blockIdx.x * blockDim.x + threadIdx.x;

    if(p < nb_verts)
    {
        const int tr_index = map_transfos[p];
        const Transfo tr = transfos[tr_index];

        const float4  tmp    = in_alpha_beta[p];
        const Vec3_cu beta   = Vec3_cu(tmp.x, tmp.y, tmp.z);
        const Vec3_cu beta_t = tr * beta;

        out_alpha_beta[p] = make_float4(beta_t.x,beta_t.y,beta_t.z,tmp.w);

        const float4   tmp2      = in_vertices[p];
        const Point_cu point     = Point_cu(tmp2.x, tmp2.y, tmp2.z);
        const Point_cu point_t   = tr * point;

        out_vertices[p] = make_float4(point_t.x, point_t.y, point_t.z, tmp.w);
    }
}

// -----------------------------------------------------------------------------

/// Transform each vertex of each rbf primitive
/// @param d_transform Map for a bone parent index its rigid transformation
/// (tab[parent[ith_bone]] = ith_bone_transformation)
void hrbf_transform(const Cuda_utils::Device::Array<Transfo>& d_transform,
                    const Cuda_utils::DA_int& d_map_transfos)
{
    if(HRBF_env::d_init_points.size() == 0) return;

    const int block_size = 16;
    const int grid_size  =
            (HRBF_env::d_init_points.size() + block_size - 1) / block_size;

    HRBF_env::unbind();

    hrbf_transform_ker
            <<<grid_size, block_size >>>
            (HRBF_env::d_init_points.size(),
             HRBF_env::d_init_points.ptr(),
             HRBF_env::d_init_alpha_beta.ptr(),
             d_map_transfos.ptr(),
             d_transform.ptr(),
             HRBF_env::hd_points.d_ptr(),
             HRBF_env::hd_alphas_betas.d_ptr());

    HRBF_env::hd_points.update_host_mem();
    HRBF_env::hd_alphas_betas.update_host_mem();

    CUDA_CHECK_ERRORS();

    HRBF_env::bind();
}

}// END HRBF_ENV NAMESPACE =====================================================

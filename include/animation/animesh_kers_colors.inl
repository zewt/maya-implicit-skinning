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
#include "mesh.hpp"

/**
 * @file animesh_kers_colors.inl
 * @brief kernels that needs to be in the main cuda programm file which is:
 * 'cuda_main_kernels.cu'
*/

#if !defined(SKELETON_ENV_EVALUATOR_HPP__)
#error "You must include skeleton_env_evaluator.hpp before the inclusion of 'animesh_kers_colors.inl'"
#endif

// =============================================================================
namespace Animesh_colors{
// =============================================================================

__global__
void gradient_potential_colors_kernel(float4* d_colors,
                                      const Mesh::Packed_data* d_map,
                                      Vec3_cu* vertices,
                                      int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        Vec3_cu grad;

        Skeleton_env::compute_potential<Skeleton_env::Std_bone_eval>(Convs::to_point(vertices[p]), grad);
        grad.normalize();
        float r = grad.x;
        float g = grad.y;
        float b = grad.z;

        if(isnan(r) || isnan(g) || isnan(b))
            r = g = b = 1.f;
        else if( grad.norm() < 0.0001f)
            r = g = b = 0.f;
        else
        {
            grad.normalize();

            // DEBUG
#if 1
            r = -grad.x * 0.5f + 0.5f;
            g = -grad.y * 0.5f + 0.5f;
            b = -grad.z * 0.5f + 0.5f;
#else
            r = g = b = 1.f;

            r = (normals[p].dot(-grad) < 0.f) ? 0.f : 1.f;
#endif

        }

        for(int i=0; i < d_map[p].nb_ocurrence; i++)
        {
            int idx = d_map[p].idx_data_unpacked + i;
            d_colors[idx] = make_float4(r ,g, b, 0.99f);
        }
    }
}

}// END ANIMESH_COLORS =========================================================

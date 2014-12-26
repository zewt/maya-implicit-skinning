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
#ifndef HRBF_ENV_TEX_HPP__
#define HRBF_ENV_TEX_HPP__

#include "cuda_compiler_interop.hpp"
#include "hrbf_env.hpp"
#include "point_cu.hpp"
#include "vec3_cu.hpp"

/** This file contains functions that handle hermite rbf primitive data into
    CUDA textures. It MUST be included in CUDA main program file.

    In order to add a hermite rbf instance to texture one has to use
    add_hrbf_instance_to_tex() @see rbf_hermite_env.hpp.

    Fetching the data is done with fetch_xxx(data_index, hrbf_ID) function
    where hrbf_ID is the identifier returned by add_hrbf_instance_to_tex() and
    data_index the index element to fetch.

    HRBF_Env::init_tex(), HRBF_Env::bind(), HRBF_Env::unbind() should not be
    used. Binding is handle inside add_hrbf_instance_to_tex()
*/

// =============================================================================
namespace HRBF_env{
// =============================================================================

texture<float4, 1, cudaReadModeElementType> tex_points;
/// First three floats represents the beta vector, last float is the alpha scalar
texture<float4, 1, cudaReadModeElementType> tex_alphas_betas;
/// tex_offset.x the offset and tex_offset.y the instance size
texture<int2, 1, cudaReadModeElementType> tex_offset;

texture<float, 1, cudaReadModeElementType> tex_radius;

// -----------------------------------------------------------------------------

static void setup_tex()
{
    // tex_points setup
    tex_points.addressMode[0] = cudaAddressModeWrap;
    tex_points.addressMode[1] = cudaAddressModeWrap;
    tex_points.filterMode = cudaFilterModePoint;
    tex_points.normalized = false;
    // tex_alphas_betas setup
    tex_alphas_betas.addressMode[0] = cudaAddressModeWrap;
    tex_alphas_betas.addressMode[1] = cudaAddressModeWrap;
    tex_alphas_betas.filterMode = cudaFilterModePoint;
    tex_alphas_betas.normalized = false;
    // tex_offset setup
    tex_offset.addressMode[0] = cudaAddressModeWrap;
    tex_offset.addressMode[1] = cudaAddressModeWrap;
    tex_offset.filterMode = cudaFilterModePoint;
    tex_offset.normalized = false;
    // tex_radius setup
    tex_radius.addressMode[0] = cudaAddressModeWrap;
    tex_radius.addressMode[1] = cudaAddressModeWrap;
    tex_radius.filterMode = cudaFilterModePoint;
    tex_radius.normalized = false;
}

// -----------------------------------------------------------------------------

/// Bind hermite array data.
static void bind_local()
{
    setup_tex();
    d_offset.bind_tex( tex_offset );
    hd_alphas_betas.device_array().bind_tex( tex_alphas_betas );
    hd_points.      device_array().bind_tex( tex_points       );
    hd_radius.      device_array().bind_tex( tex_radius       );
}

// -----------------------------------------------------------------------------

static void unbind_local()
{
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_points)       );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_alphas_betas) );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_offset)       );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_radius)       );
}

// -----------------------------------------------------------------------------

/// @return the instance radius to transform from global to compact support
IF_CUDA_DEVICE_HOST static inline
float fetch_radius(int id_instance)
{
    #ifdef __CUDA_ARCH__
    return tex1Dfetch(tex_radius, id_instance);
    #else
    return hd_radius[id_instance];
    #endif
}

// -----------------------------------------------------------------------------

/// @return the instance offset in x and size in y
IF_CUDA_DEVICE_HOST static inline
int2 fetch_inst_size_and_offset(int id_instance)
{
    #ifdef __CUDA_ARCH__
    return tex1Dfetch(tex_offset, id_instance  );
    #else
    return h_offset[id_instance];
    #endif
}

// -----------------------------------------------------------------------------

/// fetch at the same time points and weights (more efficient than functions below)
/// @param raw_idx The raw index to fetch directly from the texture.
/// raw_idx equals the offset plus the indice of the fetched sample
/// @return the alpha weight
IF_CUDA_DEVICE_HOST inline static
float fetch_weights_point(Vec3_cu& beta,
                          Point_cu& point,
                          int raw_idx)
{
    #ifdef __CUDA_ARCH__
    float4 tmp  = tex1Dfetch(tex_alphas_betas, raw_idx);
    float4 tmp2 = tex1Dfetch(tex_points, raw_idx);
    #else
    float4 tmp  = hd_alphas_betas[raw_idx];
    float4 tmp2 = hd_points[raw_idx];
    #endif
    beta  = Vec3_cu (tmp. x, tmp. y, tmp. z);
    point = Point_cu(tmp2.x, tmp2.y, tmp2.z);
    return tmp.w;
}

}// END HRBF_ENV NAMESPACE =====================================================

#endif // HRBF_ENV_TEX_HPP__

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

#if defined(__CUDACC__)
extern texture<float4, 1, cudaReadModeElementType> tex_points;
/// First three floats represents the beta vector, last float is the alpha scalar
extern texture<float4, 1, cudaReadModeElementType> tex_alphas_betas;
/// tex_offset.x the offset and tex_offset.y the instance size
extern texture<int2, 1, cudaReadModeElementType> tex_offset;

extern texture<float, 1, cudaReadModeElementType> tex_radius;
#endif

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
float fetch_radius(int id_instance)
{
    #ifdef __CUDA_ARCH__
    return tex1Dfetch(tex_radius, id_instance);
    #else
    return hd_radius[id_instance];
    #endif
}

IF_CUDA_DEVICE_HOST static inline
int2 fetch_inst_size_and_offset(int id_instance)
{
    #ifdef __CUDA_ARCH__
    return tex1Dfetch(tex_offset, id_instance  );
    #else
    return h_offset[id_instance];
    #endif
}

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

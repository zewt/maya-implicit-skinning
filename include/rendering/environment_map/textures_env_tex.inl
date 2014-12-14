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
#include "macros.hpp"


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// =============================================================================
namespace Textures_env{
// =============================================================================

__device__
float4 sample_envmap(float u, float v){ return tex2D(g_tex_envmap,u,v); }

// -----------------------------------------------------------------------------

__device__
float4 sample_envmap(const Vec3_cu& d){
    float sintheta = -d.y;
    float costheta = sqrtf(1.f - sintheta * sintheta);
    float iu = 1.f / costheta;
    float cosphi = d.x * iu;
    float sinphi = d.z * iu;
    float phi = atan2f(sinphi, cosphi)/(2*M_PI) + 0.5;
    phi = fminf(fmaxf(0.f,phi),1.f);
    float theta = atan2f(sintheta,costheta)/M_PI + 0.5f;
    theta = fminf(fmaxf(0.f,theta),1.f);
    return tex2D(g_tex_envmap, phi, theta);
}

// -----------------------------------------------------------------------------

__device__
float4 sample_light_envmap(float u, float v){
    return tex2D(g_tex_light_envmap,u,v);
}

// -----------------------------------------------------------------------------

__device__
float sample_extrusion(int n, float u, float v){
    return (n==0)?tex2D(g_tex_extrusion,u,v):
                  tex2D(g_tex_extrusion2,u,v);
}

// -----------------------------------------------------------------------------

__device__
float2 sample_extrusion_gradient(int n, float u, float v){
    return (n==0)?tex2D(g_tex_extrusion_gradient,u,v):
                  tex2D(g_tex_extrusion2_gradient,u,v);
}

// -----------------------------------------------------------------------------

void init()
{

    if( !img_ok) return;

    //define environment map texture parameters
    g_tex_envmap.normalized = true;
    g_tex_envmap.filterMode = cudaFilterModeLinear;
    g_tex_envmap.addressMode[0] = cudaAddressModeClamp;
    g_tex_envmap.addressMode[1] = cudaAddressModeClamp;

    //define environment map texture parameters
    g_tex_light_envmap.normalized = true;
    g_tex_light_envmap.filterMode = cudaFilterModeLinear;
    g_tex_light_envmap.addressMode[0] = cudaAddressModeClamp;
    g_tex_light_envmap.addressMode[1] = cudaAddressModeClamp;

    g_tex_blob.normalized = true;
    g_tex_blob.filterMode = cudaFilterModeLinear;
    g_tex_blob.addressMode[0] = cudaAddressModeWrap;
    g_tex_blob.addressMode[1] = cudaAddressModeWrap;

    g_tex_extrusion.normalized = true;
    g_tex_extrusion.filterMode = cudaFilterModeLinear;
    g_tex_extrusion.addressMode[0] = cudaAddressModeClamp;
    g_tex_extrusion.addressMode[1] = cudaAddressModeClamp;

    g_tex_extrusion_gradient.normalized = true;
    g_tex_extrusion_gradient.filterMode = cudaFilterModeLinear;
    g_tex_extrusion_gradient.addressMode[0] = cudaAddressModeClamp;
    g_tex_extrusion_gradient.addressMode[1] = cudaAddressModeClamp;

    g_tex_extrusion2.normalized = true;
    g_tex_extrusion2.filterMode = cudaFilterModeLinear;
    g_tex_extrusion2.addressMode[0] = cudaAddressModeClamp;
    g_tex_extrusion2.addressMode[1] = cudaAddressModeClamp;

    g_tex_extrusion2_gradient.normalized = true;
    g_tex_extrusion2_gradient.filterMode = cudaFilterModeLinear;
    g_tex_extrusion2_gradient.addressMode[0] = cudaAddressModeClamp;
    g_tex_extrusion2_gradient.addressMode[1] = cudaAddressModeClamp;


    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<uchar4>();

    const int lex = envmapx / light_envmap_downscale;
    const int ley = envmapy / light_envmap_downscale;
    CUDA_SAFE_CALL(cudaMallocArray(&array_envmap, &cfd, envmapx, envmapy));
    CUDA_SAFE_CALL(cudaMallocArray(&array_light_envmap, &cfd, lex, ley));
    CUDA_SAFE_CALL(cudaMemcpyToArray(array_envmap, 0, 0,img_envmap,	envmapx * envmapy * sizeof(int),	cudaMemcpyHostToDevice));
    CUDA_SAFE_CALL(cudaMemcpyToArray(array_light_envmap, 0, 0,img_light_envmap,	lex*ley * sizeof(int),	cudaMemcpyHostToDevice));
    CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_envmap, array_envmap));
    CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_light_envmap, array_light_envmap));

    if(blob_ok)
    {
        CUDA_SAFE_CALL(cudaMallocArray(&blob_tex, &cfd, blobx, bloby));
        CUDA_SAFE_CALL(cudaMemcpyToArray(blob_tex, 0, 0, blob_img,	blobx * bloby * sizeof(int),	cudaMemcpyHostToDevice));
        CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_blob, blob_tex));
    }

    if(extrusion_ok)
    {
        for(int n = 0; n < 2; n++)
        {
            cudaChannelFormatDesc cfd2 = cudaCreateChannelDesc<float>();
            CUDA_SAFE_CALL(cudaMallocArray(&array_extrusion_mask[n], &cfd2, extrusionx[n], extrusiony[n]));
            CUDA_SAFE_CALL(cudaMemcpyToArray(array_extrusion_mask[n], 0, 0, img_extrusion_mask[n],	extrusionx[n] * extrusiony[n] * sizeof(float),	cudaMemcpyHostToDevice));
            if(n == 0){
                CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_extrusion, array_extrusion_mask[n]));
            } else {
                CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_extrusion2, array_extrusion_mask[n]));
            }


            cfd2 = cudaCreateChannelDesc<float2>();
            CUDA_SAFE_CALL(cudaMallocArray(&array_extrusion_gradient[n], &cfd2, extrusionx[n], extrusiony[n]));
            CUDA_SAFE_CALL(cudaMemcpyToArray(array_extrusion_gradient[n], 0, 0, img_extrusion_gradient[n],	extrusionx[n] * extrusiony[n] * sizeof(float2),	cudaMemcpyHostToDevice));
            if(n == 0){
                CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_extrusion_gradient, array_extrusion_gradient[n]));
            } else {
                CUDA_SAFE_CALL(cudaBindTextureToArray(g_tex_extrusion2_gradient, array_extrusion_gradient[n]));
            }
        }
    }

    bind_ok = true;
}

}// END ImgTextures NAMESPACE ==================================================


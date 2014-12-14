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
#ifndef TEXTURES_ENV_TEX_HPP__
#define TEXTURES_ENV_TEX_HPP__

#include "textures_env.hpp"

//texture for environment map:
texture<uchar4, 2, cudaReadModeNormalizedFloat> g_tex_envmap;
texture<uchar4, 2, cudaReadModeNormalizedFloat> g_tex_light_envmap;
texture<uchar4, 2, cudaReadModeNormalizedFloat> g_tex_blob;
texture<float, 2, cudaReadModeElementType> g_tex_extrusion;
texture<float2, 2, cudaReadModeElementType> g_tex_extrusion_gradient;
texture<float, 2, cudaReadModeElementType> g_tex_extrusion2;
texture<float2, 2, cudaReadModeElementType> g_tex_extrusion2_gradient;

// -----------------------------------------------------------------------------
namespace Textures_env{
// -----------------------------------------------------------------------------

    __device__
    float4 sample_envmap(float u, float v);

    __device__
    float4 sample_envmap(const Vec3_cu& d);

    __device__
    float4 sample_light_envmap(float u, float v);

    __device__
    float4 sample_envmap(const Vec3_cu& d);

    __device__
    float sample_extrusion(int n, float u, float v);

    __device__
    float2 sample_extrusion_gradient(int n, float u, float v);

    void init();

}// END Textures_env -----------------------------------------------------------

#include "textures_env_tex.inl"

#endif // TEXTURES_ENV_TEX_HPP__

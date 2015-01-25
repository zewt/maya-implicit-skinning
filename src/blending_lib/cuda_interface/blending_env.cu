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
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <deque>

#include "constants.hpp"
#include "blending_env.hpp"
#include "blending_lib/controller.hpp"
#include "blending_lib/generator.hpp"
#include "class_saver.hpp"
#include "timer.hpp"
#include "std_utils.hpp"

#include "grid3_cu.hpp"

// From n_ary.hpp
void init_nary_operators();

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// =============================================================================
namespace Blending_env{
// =============================================================================

using namespace Cuda_utils;

bool binded;

/// @name List of controllers instances
/// @{
cudaArray*           d_controllers = 0;
/// The controllers stored in 2D contiguously in host mem
/// (with their padding values)
Host::Array<float2>  h_controllers;

/// List of controllers without padding
std::deque< IBL::float2* > list_controllers;


/// Tells is the ith controller instance is used or empty
/// h_ctrl_active[ith_ctrl_instance] = is_used
std::deque<bool> h_ctrl_active;
/// @}

int nb_instances = 0;

/// Magnitude of the bulge in contact :
/// Value is to be between [0.0, 0.9] outside the operator doesn't
/// behave well
float  h_magnitude_3D_bulge = 0.7f;
float* d_magnitude_3D_bulge = 0;

/// N of the ricci operator
float  h_n_3D_ricci = 1.f;
float* d_n_3D_ricci = 0;

cudaArray* d_hyperbola_profile = 0;
cudaArray* d_hyperbola_normals_profile = 0;

/// precomputed profile function for smooth union
float*  h_hyperbola_profile = 0;
float2* h_hyperbola_normals_profile = 0;

/// precomputed profile function for the bulge in contact
//{
cudaArray* d_bulge_profile = 0;
cudaArray* d_bulge_profile_normals = 0;

float*  h_bulge_profile = 0;
float2* h_bulge_normals_profile = 0;
//}

/// precomputed profile function for the 4D bulge in contact
//@{
cudaArray* d_bulge_4D_profiles = 0;
cudaArray* d_bulge_4D_profiles_normals = 0;
//@}

/// precomputed profile function for the 4D ricci
//@{
cudaArray* d_ricci_4D_profiles = 0;
cudaArray* d_ricci_4D_profiles_normals = 0;
//@}

/// 3D Bulge in contact with parameterizable strength
//@{
Cuda_utils::Device::CuArray<float>   d_block_3D_bulge;
Cuda_utils::Device::CuArray<float2>  d_block_3D_bulge_gradient;
//@}

/// 3D ricci with parameterizable N
//@{
Cuda_utils::Device::CuArray<float>   d_block_3D_ricci;
Cuda_utils::Device::CuArray<float2>  d_block_3D_ricci_gradient;
//@}

cudaArray* d_global_controller = 0;
IBL::Ctrl_setup globale_ctrl_shape;
const int nb_samples = NB_SAMPLES;

bool allocated = false;

// Precomputed opening function
float* pan_hyperbola = 0;
cudaArray* d_pan_hyperbola = 0;

texture<float, 1, cudaReadModeElementType>  profile_hyperbola_tex;
texture<float2, 1, cudaReadModeElementType> profile_hyperbola_normals_tex;
texture<float, 1, cudaReadModeElementType>  profile_bulge_tex;
texture<float2, 1, cudaReadModeElementType> profile_bulge_normals_tex;
texture<float, 1, cudaReadModeElementType>  profiles_bulge_4D_tex;
texture<float2, 1, cudaReadModeElementType> profiles_bulge_4D_normals_tex;
texture<float, 1, cudaReadModeElementType>  profiles_ricci_4D_tex;
texture<float2, 1, cudaReadModeElementType> profiles_ricci_4D_normals_tex;
texture<float, 1, cudaReadModeElementType>  opening_hyperbola_tex;
texture<float, 1, cudaReadModeElementType>  magnitude_3D_bulge_tex;
texture<float, 3, cudaReadModeElementType>  openable_bulge_4D_tex;
texture<float2, 3, cudaReadModeElementType> openable_bulge_4D_gradient_tex;
texture<float, 1, cudaReadModeElementType>  n_3D_ricci_tex;
texture<float, 3, cudaReadModeElementType>  openable_ricci_4D_tex;
texture<float2, 3, cudaReadModeElementType> openable_ricci_4D_gradient_tex;
texture<float2, 1, cudaReadModeElementType> global_controller_tex;
texture<float2, 2, cudaReadModeElementType> tex_controllers;
texture<int4  , 1, cudaReadModeElementType> tex_pred_operators_idx_offsets;
texture<int   , 1, cudaReadModeElementType> tex_pred_operators_id;
texture<float , 3, cudaReadModeElementType> tex_operators_values;
texture<float2, 3, cudaReadModeElementType> tex_operators_grads;

//------------------------------------------------------------------------------

#include <sys/stat.h>
#include <sys/types.h>

// Work around Windows bugs:
#if defined(WIN32)
#include <windows.h>
#include <direct.h>
#define mkdir(path, mode) _mkdir(path)
#endif

std::string get_cache_dir() {
    std::string dir;
#if defined(WIN32)
    char tmp[MAX_PATH+1];
    GetTempPath(sizeof(tmp), tmp);
    dir = tmp;
#else
    // This is untested.
    dir = getenv("HOME") + "/.implicit/";
#endif

    dir += "implicit/";
    mkdir(dir.c_str(), 0755);
    return dir;
}

/// @param src_vals host array to be copied. 3D values are stored linearly
/// src_vals[x + y*width + z*width*height] = [x][y][z];
/// @param d_dst_values device array to stores and allocate the values from host
template<class T>
void allocate_and_copy_3D_array(int3 size, T* h_src_vals, cudaArray*& d_dst_values)
{
    if(d_dst_values != 0) Cuda_utils::free_d(d_dst_values);

    // Allocate on device
    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<T>();
    cudaExtent volumeSize = make_cudaExtent(size.x, size.y, size.z);
    CUDA_SAFE_CALL(cudaMalloc3DArray(&d_dst_values, &cfd, volumeSize) );

    // copy data to 3D array
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr   = make_cudaPitchedPtr((void*)h_src_vals, volumeSize.width*sizeof(T), volumeSize.width, volumeSize.height);
    copyParams.dstArray = d_dst_values;
    copyParams.extent   = volumeSize;
    copyParams.kind     = cudaMemcpyHostToDevice;
    CUDA_SAFE_CALL( cudaMemcpy3D(&copyParams) );
}

// -----------------------------------------------------------------------------

/// @param src_vals host array to be copied.
/// @param d_dst_values device array to stores and allocate the values from host
template<class T>
void allocate_and_copy_1D_array(int size, T* h_src_vals, cudaArray*& d_dst_values)
{
    if(d_dst_values != 0) Cuda_utils::free_d(d_dst_values);

    int data_size = size * sizeof(T);

    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMallocArray(&d_dst_values, &cfd, size, 1));
    CUDA_SAFE_CALL(cudaMemcpyToArray(d_dst_values, 0, 0, h_src_vals, data_size, cudaMemcpyHostToDevice));
}


void bind()
{
    binded = true;
    if(allocated){

        // Openings ---------------
        opening_hyperbola_tex.normalized = false;
        opening_hyperbola_tex.addressMode[0] = cudaAddressModeClamp;
        opening_hyperbola_tex.addressMode[1] = cudaAddressModeClamp;
        opening_hyperbola_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(opening_hyperbola_tex, d_pan_hyperbola));
        // END Openings -----------

        // Profiles ---------------
        profile_hyperbola_tex.normalized = false;
        profile_hyperbola_tex.addressMode[0] = cudaAddressModeClamp;
        profile_hyperbola_tex.addressMode[1] = cudaAddressModeClamp;
        profile_hyperbola_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_hyperbola_tex, d_hyperbola_profile));

        profile_hyperbola_normals_tex.normalized = false;
        profile_hyperbola_normals_tex.addressMode[0] = cudaAddressModeClamp;
        profile_hyperbola_normals_tex.addressMode[1] = cudaAddressModeClamp;
        profile_hyperbola_normals_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_hyperbola_normals_tex, d_hyperbola_normals_profile));

        profile_bulge_tex.normalized = false;
        profile_bulge_tex.addressMode[0] = cudaAddressModeClamp;
        profile_bulge_tex.addressMode[1] = cudaAddressModeClamp;
        profile_bulge_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_bulge_tex, d_bulge_profile));

        profile_bulge_normals_tex.normalized = false;
        profile_bulge_normals_tex.addressMode[0] = cudaAddressModeClamp;
        profile_bulge_normals_tex.addressMode[1] = cudaAddressModeClamp;
        profile_bulge_normals_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_bulge_normals_tex, d_bulge_profile_normals));

        // 4D BULGE --------------
        if(d_magnitude_3D_bulge)
        {
            magnitude_3D_bulge_tex.normalized = false;
            magnitude_3D_bulge_tex.addressMode[0] = cudaAddressModeWrap;
            magnitude_3D_bulge_tex.addressMode[1] = cudaAddressModeWrap;
            magnitude_3D_bulge_tex.filterMode = cudaFilterModePoint;
            CUDA_SAFE_CALL(cudaBindTexture(0, magnitude_3D_bulge_tex, d_magnitude_3D_bulge, sizeof(float)));
        }

        if(d_bulge_4D_profiles)
        {
            profiles_bulge_4D_tex.normalized = false;
            profiles_bulge_4D_tex.addressMode[0] = cudaAddressModeClamp;
            profiles_bulge_4D_tex.addressMode[1] = cudaAddressModeClamp;
            profiles_bulge_4D_tex.filterMode = cudaFilterModeLinear;
            CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_bulge_4D_tex, d_bulge_4D_profiles));
        }

        if(d_bulge_4D_profiles_normals)
        {
            profiles_bulge_4D_normals_tex.normalized = false;
            profiles_bulge_4D_normals_tex.addressMode[0] = cudaAddressModeClamp;
            profiles_bulge_4D_normals_tex.addressMode[1] = cudaAddressModeClamp;
            profiles_bulge_4D_normals_tex.filterMode = cudaFilterModeLinear;
            CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_bulge_4D_normals_tex, d_bulge_4D_profiles_normals));
        }

        openable_bulge_4D_tex.normalized = false;
        openable_bulge_4D_tex.addressMode[0] = cudaAddressModeClamp;
        openable_bulge_4D_tex.addressMode[1] = cudaAddressModeClamp;
        openable_bulge_4D_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_bulge.bind_tex(openable_bulge_4D_tex);

        openable_bulge_4D_gradient_tex.normalized = false;
        openable_bulge_4D_gradient_tex.addressMode[0] = cudaAddressModeClamp;
        openable_bulge_4D_gradient_tex.addressMode[1] = cudaAddressModeClamp;
        openable_bulge_4D_gradient_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_bulge_gradient.bind_tex(openable_bulge_4D_gradient_tex);
        // END 4D BULGE --------------

        // 4D RICCI --------------
        if(d_n_3D_ricci)
        {
            n_3D_ricci_tex.normalized = false;
            n_3D_ricci_tex.addressMode[0] = cudaAddressModeWrap;
            n_3D_ricci_tex.addressMode[1] = cudaAddressModeWrap;
            n_3D_ricci_tex.filterMode = cudaFilterModePoint;
            CUDA_SAFE_CALL(cudaBindTexture(0, n_3D_ricci_tex, d_n_3D_ricci, sizeof(float)));
        }

        profiles_ricci_4D_tex.normalized = false;
        profiles_ricci_4D_tex.addressMode[0] = cudaAddressModeClamp;
        profiles_ricci_4D_tex.addressMode[1] = cudaAddressModeClamp;
        profiles_ricci_4D_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_ricci_4D_tex, d_ricci_4D_profiles));

        profiles_ricci_4D_normals_tex.normalized = false;
        profiles_ricci_4D_normals_tex.addressMode[0] = cudaAddressModeClamp;
        profiles_ricci_4D_normals_tex.addressMode[1] = cudaAddressModeClamp;
        profiles_ricci_4D_normals_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_ricci_4D_normals_tex, d_ricci_4D_profiles_normals));

        openable_ricci_4D_tex.normalized = false;
        openable_ricci_4D_tex.addressMode[0] = cudaAddressModeClamp;
        openable_ricci_4D_tex.addressMode[1] = cudaAddressModeClamp;
        openable_ricci_4D_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_ricci.bind_tex(openable_ricci_4D_tex);

        openable_ricci_4D_gradient_tex.normalized = false;
        openable_ricci_4D_gradient_tex.addressMode[0] = cudaAddressModeClamp;
        openable_ricci_4D_gradient_tex.addressMode[1] = cudaAddressModeClamp;
        openable_ricci_4D_gradient_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_ricci_gradient.bind_tex(openable_ricci_4D_gradient_tex);
        // END 4D RICCI --------------
        // END Profiles --------------

        // Controllers ---------------
        global_controller_tex.normalized = true;
        global_controller_tex.addressMode[0] = cudaAddressModeClamp;
        global_controller_tex.addressMode[1] = cudaAddressModeClamp;
        global_controller_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(global_controller_tex, d_global_controller));

        if( d_controllers != 0)
        {
            tex_controllers.normalized = false;
            tex_controllers.addressMode[0] = cudaAddressModeClamp;
            tex_controllers.addressMode[1] = cudaAddressModeClamp;
            tex_controllers.filterMode = cudaFilterModeLinear;
            CUDA_SAFE_CALL(cudaBindTextureToArray(tex_controllers, d_controllers));
        }
        // End Controllers -----------


        // Binary 3D operators -------
        d_operators_idx_offsets.bind_tex(tex_pred_operators_idx_offsets);
        d_operators_id.bind_tex(tex_pred_operators_id);

        tex_operators_values.normalized = false;
        tex_operators_values.addressMode[0] = cudaAddressModeClamp;
        tex_operators_values.addressMode[1] = cudaAddressModeClamp;
        tex_operators_values.filterMode = cudaFilterModeLinear;
        if (d_operators_values)
            CUDA_SAFE_CALL(cudaBindTextureToArray(tex_operators_values, d_operators_values));

        tex_operators_grads.normalized = false;
        tex_operators_grads.addressMode[0] = cudaAddressModeClamp;
        tex_operators_grads.addressMode[1] = cudaAddressModeClamp;
        tex_operators_grads.filterMode = cudaFilterModeLinear;
        if (d_operators_grads)
            CUDA_SAFE_CALL(cudaBindTextureToArray(tex_operators_grads, d_operators_grads));
        // End Binary 3D operators ---
    }
}

// -----------------------------------------------------------------------------

void unbind()
{
    binded = false;
    if(!allocated)
        return;
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_hyperbola_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(opening_hyperbola_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_hyperbola_normals_tex)     );
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_bulge_tex)                 );
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_bulge_normals_tex)         );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_bulge_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_bulge_4D_normals_tex)     );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_ricci_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_ricci_4D_normals_tex)     );
    CUDA_SAFE_CALL( cudaUnbindTexture(magnitude_3D_bulge_tex)            );
    CUDA_SAFE_CALL( cudaUnbindTexture(n_3D_ricci_tex)                    );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_bulge_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_bulge_4D_gradient_tex)    );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_ricci_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_ricci_4D_gradient_tex)    );
    CUDA_SAFE_CALL( cudaUnbindTexture(global_controller_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_controllers)                   );

    // =========================================================================
    // ======================  TEST with new env archi  ========================
    // =========================================================================
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_pred_operators_idx_offsets));
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_pred_operators_id));
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_operators_values));
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_operators_grads));
    // -----------------------------------------------------------------------------
}

// -----------------------------------------------------------------------------

void init_3D_operator(const IBL::Profile_polar::Base& profile,
                      const IBL::Opening::Base& opening,
                      float range,
                      const std::string filename,
                      bool use_cache); // see end of file

// -----------------------------------------------------------------------------

void init_3D_barths_circle_arc(bool use_cache)
{
    init_3D_operator(IBL::Profile_polar::Circle(),
                     IBL::Opening::Line(),
                     1.f,
                     "barths_circle",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_3D_barths_circle_diamond(bool use_cache)
{
    init_3D_operator(IBL::Profile_polar::Circle(),
                     IBL::Opening::Diamond(),
                     2.f,
                     "barths_circle_diamond",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_3D_clean_union(bool use_cache)
{

    IBL::Profile_polar::Discreet hyperbola_curve(h_hyperbola_profile,
                                                 (IBL::float2*)h_hyperbola_normals_profile,
                                                 NB_SAMPLES);

    typedef IBL::Opening::Discreet_hyperbola Dh;
    IBL::Opening::Discreet_hyperbola opening(Dh::OPEN_TANH);

    init_3D_operator(hyperbola_curve,
                     opening,
                     1.f,
                     "3D_clean_union",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_3D_bulge_in_contact(bool use_cache)
{
    IBL::Profile_polar::Discreet bulge_curve(h_bulge_profile,
                                             (IBL::float2*)h_bulge_normals_profile,
                                             NB_SAMPLES);

    typedef IBL::Opening::Discreet_hyperbola Dh;
    IBL::Opening::Discreet_hyperbola opening(Dh::OPEN_TANH);

    init_3D_operator(bulge_curve,
                     opening,
                     1.f,
                     "3D_bulge_in_contact",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_circle_hyperbola_open(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;
    IBL::Opening::Discreet_hyperbola opening(Dh::OPEN_TANH);

    init_3D_operator(IBL::Profile_polar::Circle(),
                     opening,
                     2.f,
                     "circle_hyperbola_open",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_circle_hyperbola_closed_h(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    init_3D_operator(IBL::Profile_polar::Circle(),
                     Dh(Dh::CLOSED_HERMITE),
                     2.f,
                     "circle_hyperbola_closed_h",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_circle_hyperbola_closed_t(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    init_3D_operator(IBL::Profile_polar::Circle(),
                     Dh(Dh::CLOSED_TANH),
                     2.f,
                     "circle_hyperbola_closed_t",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_ultimate_hyperbola_closed_h(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    init_3D_operator(IBL::Profile_polar::Discreet(h_hyperbola_profile,
                                                  (IBL::float2*)h_hyperbola_normals_profile,
                                                  NB_SAMPLES),
                     Dh(Dh::CLOSED_HERMITE),
                     2.f,
                     "ultimate_hyperbola_closed_h",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_ultimate_hyperbola_closed_t(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    init_3D_operator(IBL::Profile_polar::Discreet(h_hyperbola_profile,
                                                  (IBL::float2*)h_hyperbola_normals_profile,
                                                  NB_SAMPLES),
                     Dh(Dh::CLOSED_TANH),
                     2.f,
                     "ultimate_hyperbola_closed_t",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_bulge_hyperbola_closed_h(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    init_3D_operator(IBL::Profile_polar::Discreet(h_bulge_profile,
                                                  (IBL::float2*)h_bulge_normals_profile,
                                                  NB_SAMPLES),
                     Dh(Dh::CLOSED_HERMITE),
                     2.f,
                     "bulge_hyperbola_closed_h",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_bulge_hyperbola_closed_t(bool use_cache)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    init_3D_operator(IBL::Profile_polar::Discreet(h_bulge_profile,
                                                  (IBL::float2*)h_bulge_normals_profile,
                                                  NB_SAMPLES),
                     Dh(Dh::CLOSED_TANH),
                     2.f,
                     "bulge_hyperbola_closed_t",
                     use_cache);
}

// -----------------------------------------------------------------------------

void init_bulge_skinning_closed_t(bool use_cache)
{
    typedef IBL::Opening::Diamond Dia;

    init_3D_operator(IBL::Profile_polar::Discreet(h_bulge_profile,
                                                  (IBL::float2*)h_bulge_normals_profile,
                                                  NB_SAMPLES),
                     Dia(0.55f),
                     2.f,
                     "bulge_skinning_closed_t",
                     use_cache);
}

// -----------------------------------------------------------------------------

static int3 idx1D_to_idx3D(int idx, int sx, int sy, int sz)
{
    return make_int3( idx % sx,
                     (idx / sx) % sz,
                      idx / (sx * sy)
                     );
}

// -----------------------------------------------------------------------------

static int3 idx1D_to_idx3D(int idx, int3 len)
{
    return make_int3( idx % len.x,
                     (idx / len.x) % len.z,
                      idx / (len.x * len.y)
                     );
}

// -----------------------------------------------------------------------------

/// Copy the curve into the arrays h_vals and h_grads with padding
static void copy_4D_bulge_profile(int id,
                                  const IBL::Profile_polar::Discreet& curve,
                                  HA_float h_vals,
                                  HA_float2 h_grads)
{
    const int off = id * (NB_SAMPLES+2);
    for(int i = 0; i < NB_SAMPLES; i++)
    {
        h_vals [off+1+i] = curve.get_vals()[i];
        h_grads[off+1+i] = ((float2*)curve.get_grads())[i];
    }

    // replicate values at the extremities for the padding
    h_vals [off             ] = curve.get_vals()[0];
    h_vals [off+1+NB_SAMPLES] = curve.get_vals()[NB_SAMPLES-1];

    h_grads[off             ] = ((float2*)curve.get_grads())[0];
    h_grads[off+1+NB_SAMPLES] = ((float2*)curve.get_grads())[NB_SAMPLES-1];
}

// -----------------------------------------------------------------------------

static void replicate_padding_values(int3 grid_index,
                                     int3 grid_size,
                                     int3 block_size,
                                     HA_float& h_block_vals,
                                     HA_float2& h_block_grads)
{
#if 1
    int len = NB_SAMPLES_4D_BULGE-1;
    for(int f = 0; f < 6; f++) // For each grid's face
    {
        // Look up the face
        for(int i = 0; i < NB_SAMPLES_4D_BULGE; i++) {
            for(int j = 0; j < NB_SAMPLES_4D_BULGE; j++) {

                int3 grid_id; // <- index inside the grid
                int3 off;
                switch(f)
                {
                case 0: grid_id = make_int3(i, j, 0); off = make_int3( 0,  0, -1); break;
                case 1: grid_id = make_int3(i, 0, j); off = make_int3( 0, -1,  0); break;
                case 2: grid_id = make_int3(0, i, j); off = make_int3(-1,  0,  0); break;

                case 3: grid_id = make_int3(i  , j  , len); off = make_int3(0,0,1); break;
                case 4: grid_id = make_int3(i  , len, j  ); off = make_int3(0,1,0); break;
                case 5: grid_id = make_int3(len, i  , j  ); off = make_int3(1,0,0); break;
                }

                int3 src_3D_idx = { grid_index.x * grid_size.x + grid_id.x + 1,
                                    grid_index.y * grid_size.y + grid_id.y + 1,
                                    grid_index.z * grid_size.z + grid_id.z + 1
                                  };

                int3 dst_3D_idx = { grid_index.x * grid_size.x + grid_id.x + 1 + off.x,
                                    grid_index.y * grid_size.y + grid_id.y + 1 + off.y,
                                    grid_index.z * grid_size.z + grid_id.z + 1 + off.z
                                  };

                int src_1D_idx = src_3D_idx.x + src_3D_idx.y*block_size.x + src_3D_idx.z*block_size.x*block_size.y;
                int dst_1D_idx = dst_3D_idx.x + dst_3D_idx.y*block_size.x + dst_3D_idx.z*block_size.x*block_size.y;

                h_block_vals [dst_1D_idx] = /*0.f;*/h_block_vals [src_1D_idx];
                h_block_grads[dst_1D_idx] = /*make_float2(0.f, 0.f);*/h_block_grads[src_1D_idx];

                // TODO: take care of the diagonals of the grid for the padding
            }
        }
    }
#endif
}

// -----------------------------------------------------------------------------
#if 1
void init_4D_ricci(bool use_cache)
{
    /* Remember that we use padding to avoid interpolation between 2 3D ricci
       of different magnitudes inside the block of 3D ricci.

       Values are duplicated at the boundary of a 3D ricci grid.
       This also explains the '+2' throughout the code.
    */

    // Number of grids stored in the block for (x, y, z) directions
    const int3 block_dim = { BLOCK_4D_BULGE_LX, BLOCK_4D_BULGE_LY, BLOCK_4D_BULGE_LZ };
    const int3 grid_size = { GRIG_4D_BULGE_LX , GRIG_4D_BULGE_LY , GRIG_4D_BULGE_LZ  };

    // Total number of grids (as much as the bulge magnitude sampling)
    const int nb_grids = 30; // ###

    // FIXME: block length is misscalculated _______________________________________________________________
    int3 max_idx = idx1D_to_idx3D(nb_grids-1, block_dim);

    // Number of elements required in the block to store nb_grids
    int3 block_size = {max_idx.x * grid_size.x + grid_size.x,
                       max_idx.y * grid_size.y + grid_size.y,
                       max_idx.z * grid_size.z + grid_size.z
                      };

    assert(block_size.x <= MAX_TEX_LENGTH);
    assert(block_size.y <= MAX_TEX_LENGTH);
    assert(block_size.z <= MAX_TEX_LENGTH);

    const int block_len = block_size.x*block_size.y*block_size.z;
    // FIXME : h_block_vals[] is not initialized correctly valgring complains
    // about uninitialized mem if we do not fill it with zeros. This means
    // The loop filling it is not doing its job correctly
    HA_float  h_block_vals (block_len, 0.f                   );
    HA_float2 h_block_grads(block_len, make_float2(0.f, 0.f) );

    bool s = use_cache;

    if(use_cache)
    {
        s = s && read_array(h_block_vals.ptr() , block_len, get_cache_dir()+"/4D_ricci_vals.opc"  );
        s = s && read_array(h_block_grads.ptr(), block_len, get_cache_dir()+"/4D_ricci_grads.opc" );
    }

    HA_float  h_ricci_profiles      ((NB_SAMPLES+2)*nb_grids, 0.f);
    HA_float2 h_ricci_profiles_grads((NB_SAMPLES+2)*nb_grids, make_float2(0.f, 0.f));

    typedef IBL::Opening::Discreet_hyperbola Dh;
    IBL::Opening::Discreet_hyperbola opening(Dh::OPEN_TANH);

    for(int i = 0; i < nb_grids; i++)
    {
        double N = (double)i / 2.; // ###
        IBL::Profile_polar::Discreet ricci_curve;
        IBL::Profile::Ricci_profile ricci_profile(N);
        IBL::gen_polar_profile(ricci_curve, NB_SAMPLES, ricci_profile);

        if(!s)
        {
            float*       h_vals  = 0;
            IBL::float2* h_grads = 0;
            IBL::gen_custom_operator(ricci_curve,
                                     opening,
                                     1.f,
                                     NB_SAMPLES_4D_BULGE, NB_SAMPLES_4D_BULGE,
                                     h_vals, h_grads);

            // copy to host grid
            int3 grid_index = idx1D_to_idx3D(i, block_dim);

            for(int j = 0; j < Utils::ipow(NB_SAMPLES_4D_BULGE, 3); j++)
            {
                // index inside the grid
                int3 off = idx1D_to_idx3D(j, NB_SAMPLES_4D_BULGE, NB_SAMPLES_4D_BULGE, NB_SAMPLES_4D_BULGE);

                int3 block_3D_idx = { grid_index.x * grid_size.x + 1 + off.x,
                                      grid_index.y * grid_size.y + 1 + off.y,
                                      grid_index.z * grid_size.z + 1 + off.z
                                    };

                int block_1D_idx = block_3D_idx.x + block_3D_idx.y*block_size.x + block_3D_idx.z*block_size.x*block_size.y;

                h_block_vals [block_1D_idx] = h_vals[j];
                h_block_grads[block_1D_idx] = ((float2*)h_grads)[j];
            }

            // Duplicate values at the boundaries :
            replicate_padding_values(grid_index, grid_size, block_size, h_block_vals, h_block_grads);
            delete[] h_vals;
            delete[] h_grads;
        }

        // For each N value we store the corresponding profile into texture
        copy_4D_bulge_profile(i, ricci_curve, h_ricci_profiles, h_ricci_profiles_grads);

        delete[] ricci_curve.get_vals();
        delete[] ricci_curve.get_grads();

    }

    if(!s)
    {
        write_array(h_block_vals.ptr() , block_len, get_cache_dir()+"/4D_ricci_vals.opc"  );
        write_array(h_block_grads.ptr(), block_len, get_cache_dir()+"/4D_ricci_grads.opc" );
    }

    d_block_3D_ricci.malloc(block_size.x, block_size.y, block_size.z);
    d_block_3D_ricci.copy_from(h_block_vals);

    d_block_3D_ricci_gradient.malloc(block_size.x, block_size.y, block_size.z);
    d_block_3D_ricci_gradient.copy_from(h_block_grads);

    allocate_and_copy_1D_array((NB_SAMPLES+2)*nb_grids, h_ricci_profiles.ptr()      , d_ricci_4D_profiles        );
    allocate_and_copy_1D_array((NB_SAMPLES+2)*nb_grids, h_ricci_profiles_grads.ptr(), d_ricci_4D_profiles_normals);
}
#endif

// -----------------------------------------------------------------------------

void init_4D_bulge_in_contact(bool use_cache)
{
    /* Remember that we use padding to avoid interpolation between 2 3D bulge
       of different magnitudes inside the block of 3D bulge.

       Values are duplicated at the boundary of a 3D bulge grid.
       This also explains the '+2' throughout the code.
    */

    // Number of grids stored in the block for (x, y, z) directions
    const int3 block_dim = { BLOCK_4D_BULGE_LX, BLOCK_4D_BULGE_LY, BLOCK_4D_BULGE_LZ };
    const int3 grid_size = { GRIG_4D_BULGE_LX , GRIG_4D_BULGE_LY , GRIG_4D_BULGE_LZ  };

    // Total number of grids (as much as the bulge magnitude sampling)
    const int nb_grids = NB_SAMPLES_MAG_4D_BULGE;

    // FIXME: block length is misscalculated _______________________________________________________________
    int3 max_idx = idx1D_to_idx3D(nb_grids-1, block_dim);

    // Number of elements required in the block to store nb_grids
    int3 block_size = {max_idx.x * grid_size.x + grid_size.x,
                       max_idx.y * grid_size.y + grid_size.y,
                       max_idx.z * grid_size.z + grid_size.z
                      };

    assert(block_size.x <= MAX_TEX_LENGTH);
    assert(block_size.y <= MAX_TEX_LENGTH);
    assert(block_size.z <= MAX_TEX_LENGTH);

    const int block_len = block_size.x*block_size.y*block_size.z;
    // FIXME : h_block_vals[] is not initialized correctly valgring complains
    // about uninitialized mem if we do not fill it with zeros. This means
    // The loop filling it is not doing its job correctly
    HA_float  h_block_vals (block_len, 0.f                   );
    HA_float2 h_block_grads(block_len, make_float2(0.f, 0.f) );

    bool s = use_cache;

    if(use_cache)
    {
        s = s && read_array(h_block_vals.ptr() , block_len, get_cache_dir()+"/4D_bulge_vals.opc"  );
        s = s && read_array(h_block_grads.ptr(), block_len, get_cache_dir()+"/4D_bulge_grads.opc" );
    }

    HA_float  h_bulge_profiles      ((NB_SAMPLES+2)*nb_grids, 0.f);
    HA_float2 h_bulge_profiles_grads((NB_SAMPLES+2)*nb_grids, make_float2(0.f, 0.f));

    typedef IBL::Opening::Discreet_hyperbola Dh;
    IBL::Opening::Discreet_hyperbola opening(Dh::CLOSED_TANH);

    for(int i = 0; i < nb_grids; i++)
    {
        float mag = (float)i / (float)nb_grids;
        IBL::Profile_polar::Discreet bulge_curve;
        IBL::gen_polar_profile(bulge_curve, NB_SAMPLES, IBL::Profile::Bulge(mag));

        if(!s)
        {
            float*       h_vals  = 0;
            IBL::float2* h_grads = 0;
            IBL::gen_custom_operator(bulge_curve,
                                     opening,
                                     1.f,
                                     NB_SAMPLES_4D_BULGE, NB_SAMPLES_4D_BULGE,
                                     h_vals, h_grads);

            // copy to host grid
            int3 grid_index = idx1D_to_idx3D(i, block_dim);

            for(int j = 0; j < Utils::ipow(NB_SAMPLES_4D_BULGE, 3); j++)
            {
                // index inside the grid
                int3 off = idx1D_to_idx3D(j, NB_SAMPLES_4D_BULGE, NB_SAMPLES_4D_BULGE, NB_SAMPLES_4D_BULGE);

                int3 block_3D_idx = { grid_index.x * grid_size.x + 1 + off.x,
                                      grid_index.y * grid_size.y + 1 + off.y,
                                      grid_index.z * grid_size.z + 1 + off.z
                                    };

                int block_1D_idx = block_3D_idx.x + block_3D_idx.y*block_size.x + block_3D_idx.z*block_size.x*block_size.y;

                h_block_vals [block_1D_idx] = h_vals[j];
                h_block_grads[block_1D_idx] = ((float2*)h_grads)[j];
            }

            // Duplicate values at the boundaries :
            replicate_padding_values(grid_index, grid_size, block_size, h_block_vals, h_block_grads);
            delete[] h_vals;
            delete[] h_grads;
        }

        // For each strength value we store the corresponding profile into texture
        copy_4D_bulge_profile(i, bulge_curve, h_bulge_profiles, h_bulge_profiles_grads);

        delete[] bulge_curve.get_vals();
        delete[] bulge_curve.get_grads();

    }

    if(!s)
    {
        write_array(h_block_vals.ptr() , block_len, get_cache_dir()+"/4D_bulge_vals.opc"  );
        write_array(h_block_grads.ptr(), block_len, get_cache_dir()+"/4D_bulge_grads.opc" );
    }

    d_block_3D_bulge.malloc(block_size.x, block_size.y, block_size.z);
    d_block_3D_bulge.copy_from(h_block_vals);

    d_block_3D_bulge_gradient.malloc(block_size.x, block_size.y, block_size.z);
    d_block_3D_bulge_gradient.copy_from(h_block_grads);

    allocate_and_copy_1D_array((NB_SAMPLES+2)*nb_grids, h_bulge_profiles.ptr()      , d_bulge_4D_profiles        );
    allocate_and_copy_1D_array((NB_SAMPLES+2)*nb_grids, h_bulge_profiles_grads.ptr(), d_bulge_4D_profiles_normals);
}

// -----------------------------------------------------------------------------

void init_profile_hyperbola(bool use_cache)
{
    assert(!binded);
    int len = NB_SAMPLES;

    delete[] h_hyperbola_profile;
    delete[] h_hyperbola_normals_profile;

    float*       h_vals    = 0;
    IBL::float2* h_grads = 0;

    bool s = use_cache;

    if(use_cache)
    {
        h_vals  = new float      [len];
        h_grads = new IBL::float2[len];
        s = s && read_array(h_vals , len, get_cache_dir()+"/profile_hyperbola_vals.opc"  );
        s = s && read_array(h_grads, len, get_cache_dir()+"/profile_hyperbola_grads.opc" );
    }

    if(!s)
    {
        IBL::Profile_polar::Discreet hyperbola_curve;

        // Profile is not cached we compute it:
        IBL::gen_polar_profile(hyperbola_curve,
                               len,
                               IBL::Profile::Hyperbola());

        h_vals  = hyperbola_curve.get_vals();
        h_grads = hyperbola_curve.get_grads();
        // And save it
        write_array(h_vals , len, get_cache_dir()+"/profile_hyperbola_vals.opc"  );
        write_array(h_grads, len, get_cache_dir()+"/profile_hyperbola_grads.opc" );

    }

    h_hyperbola_profile         = h_vals;
    h_hyperbola_normals_profile = (float2*)h_grads;

    allocate_and_copy_1D_array(len, h_hyperbola_profile        , d_hyperbola_profile        );
    allocate_and_copy_1D_array(len, h_hyperbola_normals_profile, d_hyperbola_normals_profile);
}

// -----------------------------------------------------------------------------

void init_profile_bulge(bool use_cache)
{
    assert(!binded);
    int len = NB_SAMPLES;

    delete[] h_bulge_profile;
    delete[] h_bulge_normals_profile;

    float*       h_vals  = 0;
    IBL::float2* h_grads = 0;

    bool s = use_cache;

    if(use_cache)
    {
        h_vals  = new float      [len];
        h_grads = new IBL::float2[len];
        s = s && read_array(h_vals , len, get_cache_dir()+"/profile_bulge_vals.opc"  );
        s = s && read_array(h_grads, len, get_cache_dir()+"/profile_bulge_grads.opc" );
    }

    if(!s)
    {
        IBL::Profile_polar::Discreet bulge_curve;
        // Profile is not cached we compute it:
        IBL::gen_polar_profile(bulge_curve,
                               len,
                               IBL::Profile::Bulge(h_magnitude_3D_bulge) );

        h_vals  = bulge_curve.get_vals();
        h_grads = bulge_curve.get_grads();
        // And save it
        write_array(h_vals , len, get_cache_dir()+"/profile_bulge_vals.opc"  );
        write_array(h_grads, len, get_cache_dir()+"/profile_bulge_grads.opc" );

    }

    h_bulge_profile         = h_vals;
    h_bulge_normals_profile = (float2*)h_grads;

    allocate_and_copy_1D_array(len, h_bulge_profile        , d_bulge_profile         );
    allocate_and_copy_1D_array(len, h_bulge_normals_profile, d_bulge_profile_normals );
}

// -----------------------------------------------------------------------------

void init_opening_hyperbola(bool use_cache)
{
    assert(!binded);
    int len = IBL::Opening::Pan_hf::_nb_samples;

    delete[] pan_hyperbola;

    bool s = use_cache;

    if(use_cache){
        pan_hyperbola = new float[len];
        s = s && read_array(pan_hyperbola, len, get_cache_dir()+"/opening_hyperbola_vals.opc");
    }


    if(!s)
    {
        typedef IBL::Opening::Discreet_hyperbola Dh;
        IBL::Opening::Discreet_hyperbola opening(Dh::OPEN_TANH);
        // opening is not cached we compute it:

        IBL::Opening::Pan_hf::init_samples();
        delete pan_hyperbola;
        pan_hyperbola = new float[len];
        for(int i = 0; i < len; ++i)
            pan_hyperbola[i] = IBL::Opening::Pan_hf::_vals[i];

        // And save it
        write_array(pan_hyperbola, len, get_cache_dir()+"/opening_hyperbola_vals.opc");
    }

    allocate_and_copy_1D_array(len, pan_hyperbola, d_pan_hyperbola);
}

// -----------------------------------------------------------------------------

void init_global_controller()
{
    assert(!binded);
    IBL::float2* controller;
    globale_ctrl_shape = IBL::Shape::elbow();
    IBL::gen_controller(NB_SAMPLES, globale_ctrl_shape, controller);
    allocate_and_copy_1D_array(NB_SAMPLES, (float2*)controller, d_global_controller);
    delete[] controller;
}

// -----------------------------------------------------------------------------

IBL::Ctrl_setup get_global_ctrl_shape()
{
    return globale_ctrl_shape;
}

// -----------------------------------------------------------------------------

float eval_global_ctrl(float dot)
{
    IBL::Continuous::Controller ctrl(globale_ctrl_shape);
    return ctrl.eval(dot);
}

// -----------------------------------------------------------------------------

void update_3D_bulge()
{
    unbind();

    std::cout << "update samples \n..." << std::endl;
    init_profile_bulge(false);
    init_3D_bulge_in_contact(false);

    bind();
}

// -----------------------------------------------------------------------------

static void d_controllers_malloc(int2 size)
{
    assert(!binded);
    if(d_controllers != 0) CUDA_SAFE_CALL(cudaFreeArray(d_controllers));

    if(size.x * size.y > 0) malloc_2D_array<float2>(d_controllers, size);
    else                    d_controllers = 0;
}

// -----------------------------------------------------------------------------

static void d_controllers_copy_from(const Cuda_utils::Host::Array<float2>& h_a, int2 size)
{
    if( size.x * size.y > 0)
    {
        assert(!binded);
        mem_cpy_2D_htd(d_controllers, h_a.ptr(), size);
    }
}

// -----------------------------------------------------------------------------

/// @return the size of the block in (x,y) direction given the current number of
/// controllers
static int2 ctrl_max_size_2D()
{
    int nb_ctrl = list_controllers.size() - 1;

    if(nb_ctrl < 0) return make_int2(0, 0);

    int2 bidx_2D = { nb_ctrl % BLOCK_CTRL_LX, nb_ctrl / BLOCK_CTRL_LX };
    int2 s = { (bidx_2D.y == 0) ? ((bidx_2D.x+1) * GRID_CTRL_LX) : (BLOCK_CTRL_LX*GRID_CTRL_LX),
               (bidx_2D.y+1) * GRID_CTRL_LY};

    // If triggered means that to many controllers has been created.
    assert(s.x <= MAX_2D_TEX_LENGTH_X);
    assert(s.y <= MAX_2D_TEX_LENGTH_Y);
    return s;
}

// -----------------------------------------------------------------------------

static int2 ctrl_1DIdx_to_2DIdx(int id)
{
    return make_int2(id%BLOCK_CTRL_LX, id/BLOCK_CTRL_LX);
}

// -----------------------------------------------------------------------------

/// copy the 2D list of controllers 'list' in the flat representation and device
/// memory.
void update_ctrl_in_device()
{
    assert(!binded);
    int2 gsize2D = ctrl_max_size_2D();
    h_controllers.malloc( gsize2D.x * gsize2D.y );
    d_controllers_malloc( gsize2D               );

    // Do the copy of 'list_controllers' into the flatten array 'h_controllers'
    const int width = gsize2D.x;
    for(unsigned i = 0; i < list_controllers.size(); i++)
    {
        float2* ctrl = (float2*)list_controllers[i];

        // Nothing to copy we skip
        if(ctrl == 0) continue;

        int2 bidx_2D = ctrl_1DIdx_to_2DIdx( i );
        int2 gidx_2D = {bidx_2D.x * GRID_CTRL_LX, bidx_2D.y * GRID_CTRL_LY };

        for(int j = 0; j < NB_SAMPLES; j++)
        {
            int row0_idx = (gidx_2D.x + j + 1) + (gidx_2D.y  ) * width;
            int row1_idx = (gidx_2D.x + j + 1) + (gidx_2D.y+1) * width;
            int row2_idx = (gidx_2D.x + j + 1) + (gidx_2D.y+2) * width;

            float2 val = ctrl[j];
            h_controllers[row0_idx] = val;
            h_controllers[row1_idx] = val;
            h_controllers[row2_idx] = val;
        }

        // Copy extremities
        int row0_idx = (gidx_2D.x) + (gidx_2D.y  ) * width;
        int row1_idx = (gidx_2D.x) + (gidx_2D.y+1) * width;
        int row2_idx = (gidx_2D.x) + (gidx_2D.y+2) * width;
        float2 val = ctrl[0];
        // Pad start
        h_controllers[row0_idx] = val;
        h_controllers[row1_idx] = val;
        h_controllers[row2_idx] = val;

        // Pad end
        val = ctrl[NB_SAMPLES-1];
        h_controllers[row0_idx+NB_SAMPLES+1] = val;
        h_controllers[row1_idx+NB_SAMPLES+1] = val;
        h_controllers[row2_idx+NB_SAMPLES+1] = val;
    }
    // Update the device array
    d_controllers_copy_from(h_controllers, gsize2D);
}

// -----------------------------------------------------------------------------

Ctrl_id new_ctrl_instance()
{
    assert(nb_instances >= 0);

    Blending_env::unbind();

    // find the first free element
    int idx = 0;
    for(; idx < (int)h_ctrl_active.size(); idx++)
        if( !h_ctrl_active[idx] )
            break;

    // We need to allocate more memory
    if(idx == (int)h_ctrl_active.size())
    {
        list_controllers.push_back(0);
        h_ctrl_active.push_back( true );
        // Increase memory and recopy values from 'list_controllers'
        update_ctrl_in_device();
    }

    h_ctrl_active[idx] = true;
    nb_instances++;

    Blending_env::bind();
    return idx;
}

// -----------------------------------------------------------------------------

int get_nb_ctrl_instances()
{
    assert(false);
    return nb_instances;
}

// -----------------------------------------------------------------------------

void delete_ctrl_instance(Ctrl_id inst_id)
{
    assert(inst_id < (int)h_ctrl_active.size());
    assert(inst_id >= 0);
    assert(h_ctrl_active[inst_id]);
    assert(nb_instances > 0);

    // Deleted controller instances are tagged in order to
    // re-use the element for a new instance later
    h_ctrl_active[inst_id] = false;

    if(inst_id != (int)(h_ctrl_active.size()-1))
    {
        // The instance is not in the end of the list we keep memory space of
        // h_controllers and d_controllers for later use.
        nb_instances--;
        delete[] list_controllers[inst_id];
        list_controllers[inst_id] = 0;
        return;
    }

    // Instance is at the end we erase what is necessary :
    Blending_env::unbind();

    // Scan for every deleted instances at the top of the list
    int idx = h_ctrl_active.size()-1;
    while(h_ctrl_active[idx] == false)
    {
        delete[] list_controllers[idx];
        list_controllers.pop_back();
        h_ctrl_active.pop_back();

        idx--;
        if(idx < 0) break;
    }

    update_ctrl_in_device();

    Blending_env::bind();
}

// -----------------------------------------------------------------------------

void update_controller(Ctrl_id inst_id, const IBL::Ctrl_setup& shape)
{
    assert(inst_id < (int)h_ctrl_active.size());
    assert(inst_id >= 0);
    assert(h_ctrl_active[inst_id]);
    assert(nb_instances > 0);
    Blending_env::unbind();

    int2 bidx_2D = {inst_id % BLOCK_CTRL_LX , inst_id / BLOCK_CTRL_LX  };
    int2 gidx_2D = {bidx_2D.x * GRID_CTRL_LX, bidx_2D.y * GRID_CTRL_LY };
    int2 gsize2D = ctrl_max_size_2D();

    const int width = gsize2D.x;

    IBL::float2* controller = 0;
    IBL::gen_controller(NB_SAMPLES, shape, controller);

    int off = (gidx_2D.x+ 1/*padding*/) + (gidx_2D.y+1/*write values in the midlle*/) * width;

    Cuda_utils::mem_cpy_hth((IBL::float2*)(h_controllers.ptr()+off), controller, NB_SAMPLES);

    delete[] list_controllers[inst_id];
    list_controllers[inst_id] = controller;

    // Padd values :
    h_controllers[off-1           ] = h_controllers[off           ];
    h_controllers[off+NB_SAMPLES+1] = h_controllers[off+NB_SAMPLES];

    for(int i = 0; i < (NB_SAMPLES+2/*Padding*/); i++)
    {
        int row0_idx = (gidx_2D.x + i) + (gidx_2D.y  ) * width;
        int row1_idx = (gidx_2D.x + i) + (gidx_2D.y+1) * width;
        int row2_idx = (gidx_2D.x + i) + (gidx_2D.y+2) * width;

        float2 val = h_controllers[row1_idx];
        h_controllers[row0_idx] = val;
        h_controllers[row2_idx] = val;
    }

    d_controllers_copy_from(h_controllers, gsize2D);
    Blending_env::bind();
}

// -----------------------------------------------------------------------------






// =============================================================================
// ======================  TEST with new env archi  ============================
// =============================================================================

const int PADDING = 2;
const Vec3i_cu PADDING_OFFSET(PADDING/2, PADDING/2, PADDING/2);

const int NB_PRED_OPS = BINARY_3D_OPERATOR_END - BINARY_3D_OPERATOR_BEGIN -1;

/// Does the GPU memory is up to date with the CPU blending operators
bool updated = false; // TODO <- assert in device when not updated

/// Every blending operators values/gradients concatenated in one cudaArray
/// @{
cudaArray* d_operators_values = 0;
cudaArray* d_operators_grads  = 0;
/// @}

/// Every blending operators values/gradients concatenated in one CPU grid
/// @{
Grid3_cu<float>*  grid_operators_values = 0;
Grid3_cu<float2>* grid_operators_grads  = 0;
/// @}

/// this array saves the offset needed to access blending operators
/// in 'd_operators_xxx' or 'grid_operators_xxx'
/// @{
std::vector<Idx3_cu> h_operators_idx_offsets;
Cuda_utils::Device::Array<int4> d_operators_idx_offsets; // GPU mem
/// @}

/// maps operators types to their identifier.
// TODO this maps only a sub part of operators type it should map everything
// and with id=-1 for operators types which doesn't exists.
Cuda_utils::Device::Array<Op_id> d_operators_id;

/// predefined operators grids
std::vector< Grid3_cu<float>*  > h_operators_values;
std::vector< Grid3_cu<float2>* > h_operators_grads;

bool h_operators_enabling[NB_PRED_OPS] = {};

/// user customly defined operators
std::vector< Grid3_cu<float>*  > h_custom_op_vals;
std::vector< Grid3_cu<float2>* > h_custom_op_grads;

// -----------------------------------------------------------------------------

/// @param src_vals host array to be copied. 3D values are stored linearly
/// src_vals[x + y*width + z*width*height] = [x][y][z];
/// @param d_dst_values device array to stores and allocate the values from host
template<class T>
void allocate_and_copy_3D_array(Grid3_cu<T>* grid_data,
                                cudaArray*& d_dst_values)
{
    if(d_dst_values != 0) Cuda_utils::free_d( d_dst_values );
    d_dst_values = grid_data->to_gpu();
}

// -----------------------------------------------------------------------------

void init_3D_operator(const IBL::Profile_polar::Base& profile,
                      const IBL::Opening::Base& opening,
                      float range,
                      const std::string filename,
                      bool use_cache)
{
    float*       h_vals  = 0;
    IBL::float2* h_grads = 0;

    int len = (NB_SAMPLES_OCU+2)*(NB_SAMPLES_OCU+2)*(NB_SAMPLES_ALPHA+2);
    bool s = use_cache;
    Vec3i_cu size(NB_SAMPLES_OCU, NB_SAMPLES_OCU, NB_SAMPLES_ALPHA);
    Vec3i_cu pad_off(0, 0, 0);
    if(use_cache)
    {
        // Operator must be cached => get it (already padded)
        h_vals  = new float      [len];
        h_grads = new IBL::float2[len];
        s = s && read_array(h_vals , len, get_cache_dir()+"/"+filename+"_vals.opc"  );
        s = s && read_array(h_grads, len, get_cache_dir()+"/"+filename+"_grads.opc" );
    }

    if(!s)
    {
        // Operator is not cached => compute it
        IBL::gen_custom_operator(profile,
                                 opening,
                                 range,
                                 NB_SAMPLES_OCU, NB_SAMPLES_ALPHA ,
                                 h_vals, h_grads);
    }
    else {
        // as already padded
        size += Vec3i_cu(2, 2, 2);
        pad_off = PADDING_OFFSET;
    }

    // store into new grids
    Grid3_cu<float >* grid_vals  = new Grid3_cu<float >(size, h_vals          , pad_off);
    Grid3_cu<float2>* grid_grads = new Grid3_cu<float2>(size, (float2*)h_grads, pad_off);

    // if not cached : padd it as concatenation won't and save it padded
    if( !s)
    {
        // padd it as concatenation won't
        grid_vals-> padd( Vec3i_cu(PADDING, PADDING, PADDING) );
        grid_grads->padd( Vec3i_cu(PADDING, PADDING, PADDING) );
        if ( filename.size() > 0 ){
            write_array(grid_vals->get_vals().data(), len, get_cache_dir()+"/"+filename+"_vals.opc"  );
            write_array(grid_grads->get_vals().data(), len, get_cache_dir()+"/"+filename+"_grads.opc" );
        }
    }

    // record the new operator
    h_operators_values.push_back( grid_vals  );
    h_operators_grads. push_back( grid_grads );

    delete[] h_vals;
    delete[] h_grads;
}

// -----------------------------------------------------------------------------

void load_3d_predefined(bool use_cache = true)
{
    if ( h_operators_enabling[0] ) {
        init_3D_barths_circle_arc(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[1] ) {
        init_3D_barths_circle_diamond(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[2] ) {
        init_circle_hyperbola_open(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[3] ) {
        init_circle_hyperbola_closed_h(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[4] ) {
        init_circle_hyperbola_closed_t(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[5] ) {
        init_3D_clean_union(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[6] ) {
        init_ultimate_hyperbola_closed_h(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[7] ) {
        init_ultimate_hyperbola_closed_t(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[8] ) {
        init_3D_bulge_in_contact(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[9] ) {
        init_bulge_hyperbola_closed_h(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[10] ) {
        init_bulge_hyperbola_closed_t(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
    if ( h_operators_enabling[11] ) {
        init_bulge_skinning_closed_t(use_cache);
    } else {
        h_operators_values.push_back(0);
        h_operators_grads.push_back(0);
    }
}

// -----------------------------------------------------------------------------

void init_env()
{
    clean_env();
    // add all operators
    Timer t;
    bool use_cache = true;


    t.start();
    std::cout << "init profile samples skin (bulge in contact\n..." << std::endl;
    init_profile_bulge(use_cache);
    std::cout << "Done in " << t.stop() << " sec" << std::endl;
    t.start();
    std::cout << "init profile samples hyperbola\n..." << std::endl;
    init_profile_hyperbola(use_cache);
    std::cout << "Done in " << t.stop() << " sec" << std::endl;
    t.start();
    std::cout << "init samples opening func pan hyperbola\n..." << std::endl;
    init_opening_hyperbola(use_cache);
    std::cout << "Done in " << t.stop() << " sec" << std::endl;

    std::cout << "init 3D operators" << std::endl;
    load_3d_predefined(use_cache);
    std::cout << "Done in " << t.stop() << " sec" << std::endl;

    // todo with new env archi
    std::cout << "init 4D operators" << std::endl;
    std::cout << "init 4D bulge" << std::endl;
    t.start();
    init_4D_bulge_in_contact(use_cache);
    std::cout << "Done in " << t.stop() << " sec" << std::endl;
    t.start();
    std::cout << "init 4D ricci" << std::endl;
    init_4D_ricci(use_cache);
    std::cout << "Done in " << t.stop() << " sec" << std::endl;
    std::cout << "end 4D operators" << std::endl;

    std::cout <<  "Allocate and init controller\n..." << std::endl;
    init_global_controller();
    std::cout <<  "Done" << std::endl;

    Cuda_utils::malloc_d(d_magnitude_3D_bulge, 1);
    Cuda_utils::mem_cpy_htd(d_magnitude_3D_bulge, &h_magnitude_3D_bulge, 1);


    Cuda_utils::malloc_d(d_n_3D_ricci, 1);
    Cuda_utils::mem_cpy_htd(d_n_3D_ricci, &h_n_3D_ricci, 1);

    //init_nary_operators();

    allocated = true;

    // upload 3D operators to gpu
    t.start();
    std::cout << "Concatenate and upload operators to GPU" << std::endl;
    update_operators();
    std::cout << "Done in " << t.stop() << " sec" << std::endl;
}

// -----------------------------------------------------------------------------

static Vec3i_cu get_max_tex3D_lengths()
{
    int mx = -1;
    int my = -1;
    int mz = -1;

    int device_id = -1;
    cudaGetDevice( &device_id );
    CUdevice cu_device;
    CU_SAFE_CALL(cuDeviceGet(&cu_device, device_id));
    CU_SAFE_CALL(cuDeviceGetAttribute(&mx, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_WIDTH , cu_device) );
    CU_SAFE_CALL(cuDeviceGetAttribute(&my, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_HEIGHT, cu_device) );
    CU_SAFE_CALL(cuDeviceGetAttribute(&mz, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_DEPTH , cu_device) );

    return Vec3i_cu(mx, my, mz);
}

// -----------------------------------------------------------------------------

static void get_enabled_operators(std::vector<Grid3_cu<float>* >& ops_vals,
                                  std::vector<Grid3_cu<float2>*>& ops_grads)
{
    ops_vals. clear();
    ops_grads.clear();

    for(unsigned i = 0; i < h_operators_values.size(); ++i){
        if ( h_operators_enabling[i] ) {
            ops_vals. push_back( h_operators_values[i] );
            ops_grads.push_back( h_operators_grads [i] );
        }
    }

    for(unsigned i = 0; i < h_custom_op_vals.size(); ++i){
        if ( h_custom_op_vals[i] ){
            ops_vals. push_back( h_custom_op_vals [i] );
            ops_grads.push_back( h_custom_op_grads[i] );
        }
    }
}

// -----------------------------------------------------------------------------

static void update_map_operators_type_to_id()
{
    std::vector<Op_id> pred_id(NB_PRED_OPS, -1);
    int id = 0;
    for( int i = 0; i < NB_PRED_OPS; ++i){
        if(h_operators_enabling[i])
        {
            pred_id[i] = id;
            ++id;
        }
    }

    d_operators_id.malloc( pred_id.size() );
    d_operators_id.copy_from( pred_id );
}

// -----------------------------------------------------------------------------

int get_nb_predefined_enabled()
{
    int acc = 0;
    for(int i = 0; i < NB_PRED_OPS; ++i)
        if(h_operators_enabling[i])
            acc++;

    return acc;
}

// -----------------------------------------------------------------------------

void update_operators()
{
    unbind();
    assert( !binded );
    // ensure vals and grads for all operators
    assert( h_operators_values.size() == h_operators_grads.size() );
    assert( h_custom_op_vals.  size() == h_custom_op_grads.size() );

    updated = true;

    Vec3i_cu len_max = get_max_tex3D_lengths();

    std::vector< Grid3_cu<float>*  > all_op_vals;
    std::vector< Grid3_cu<float2>* > all_op_grads;
    get_enabled_operators(all_op_vals, all_op_grads);

    // erase concatenated grids
    delete grid_operators_values; grid_operators_values = 0;
    delete grid_operators_grads;  grid_operators_grads  = 0;
    // Erase associated offsets
    h_operators_idx_offsets.clear();

    if( all_op_vals.size() == 0 )
    {
        Cuda_utils::free_d(d_operators_values);
        Cuda_utils::free_d(d_operators_grads);
        d_operators_idx_offsets.erase();
        d_operators_id.erase();
        return;
    }

    std::vector<Idx3_cu> dummy_idx;

    // concatenate custom & predefined grid vectors
    grid_operators_values = new Grid3_cu<float >( all_op_vals , len_max, h_operators_idx_offsets );
    grid_operators_grads  = new Grid3_cu<float2>( all_op_grads, len_max, dummy_idx );

    std::cout << "Grid operators size: ";
    grid_operators_values->size().print();
    assert( Std_utils::equal(dummy_idx, h_operators_idx_offsets) );

    // Upload to GPU
    allocate_and_copy_3D_array(grid_operators_values, d_operators_values);
    allocate_and_copy_3D_array(grid_operators_grads , d_operators_grads );

    // TODO: to be deleted seems wrong
//    int nb_predefined = get_nb_predefined_enabled();

    int4 default_idx = make_int4(0, 0, 0, 0);
    std::vector<int4> indices( h_operators_values.size() + h_custom_op_vals.size(), default_idx );

    unsigned int i; int j=0;
    for(i = 0; i < h_operators_values.size(); ++i)
        if (h_operators_enabling[i]){
            indices[j] = h_operators_idx_offsets[j].to_int4();
            ++j;
        }

    unsigned int k = i;
    for(unsigned i = 0; i < h_custom_op_vals.size(); ++i)
        if (h_custom_op_vals[i]){
            indices[k+i] = h_operators_idx_offsets[j].to_int4();
            ++j;
        }


    // upload idx
    d_operators_idx_offsets.malloc( indices.size() );
    d_operators_idx_offsets.copy_from( indices );

    update_map_operators_type_to_id();
    bind();
}

// -----------------------------------------------------------------------------

void clean_env()
{
    unbind();

    // free controllers -----------------
    for(unsigned i = 0; i < list_controllers.size(); i++)
        delete[] list_controllers[i];
    h_controllers.erase();
    h_ctrl_active.clear();
    list_controllers.clear();
    nb_instances = 0;

    // free profiles -----------------
    delete[] h_hyperbola_profile;
    delete[] h_hyperbola_normals_profile;
    delete[] h_bulge_profile;
    delete[] h_bulge_normals_profile;
    delete[] pan_hyperbola;
    h_hyperbola_profile = 0;
    h_hyperbola_normals_profile = 0;
    h_bulge_profile = 0;
    h_bulge_normals_profile = 0;
    pan_hyperbola = 0;

    // free binary 3D operators -----------------
    for(unsigned i = 0; i < h_operators_values.size(); ++i) {
        delete h_operators_values[i];
        delete h_operators_grads[i];
    }
    h_operators_values.clear();
    h_operators_grads.clear();

    for(unsigned i = 0; i < h_custom_op_vals.size(); ++i) {
        delete h_custom_op_vals[i];
        delete h_custom_op_grads[i];
    }
    h_custom_op_vals.clear();
    h_custom_op_grads.clear();

    h_operators_idx_offsets.clear();
    delete grid_operators_values;
    delete grid_operators_grads;
    grid_operators_values = 0;
    grid_operators_grads = 0;
    d_operators_idx_offsets.erase();
    d_operators_id.erase();

    // free gpu memory -----------------
    if(allocated){
        // controllers
        Cuda_utils::free_d(d_controllers);
        Cuda_utils::free_d(d_global_controller);
        // profiles
        Cuda_utils::free_d(d_hyperbola_profile);
        Cuda_utils::free_d(d_hyperbola_normals_profile);
        Cuda_utils::free_d(d_bulge_profile);
        Cuda_utils::free_d(d_bulge_profile_normals);
        Cuda_utils::free_d(d_bulge_4D_profiles);
        Cuda_utils::free_d(d_bulge_4D_profiles_normals);
        Cuda_utils::free_d(d_ricci_4D_profiles);
        Cuda_utils::free_d(d_ricci_4D_profiles_normals);
        Cuda_utils::free_d(d_pan_hyperbola);
        // binary 3D operators
        Cuda_utils::free_d(d_operators_values);
        Cuda_utils::free_d(d_operators_grads);
        d_operators_values = 0;
        d_operators_grads = 0;
        // binary 4D operators
        Cuda_utils::free_d(d_magnitude_3D_bulge);
        Cuda_utils::free_d(d_n_3D_ricci);
        d_block_3D_bulge.erase();
        d_block_3D_bulge_gradient.erase();
        d_block_3D_ricci.erase();
        d_block_3D_ricci_gradient.erase();
        allocated = false;
    }
}

// -----------------------------------------------------------------------------

Op_id new_op_instance(const IBL::Profile_polar::Base& profile,
                      const IBL::Opening::Base& opening)
{
    float*       h_vals  = 0;
    IBL::float2* h_grads = 0;
    // Operator is not cached => compute it
    IBL::gen_custom_operator(profile, opening, 2.f,
                             NB_SAMPLES_OCU, NB_SAMPLES_ALPHA ,
                             h_vals, h_grads);

    // store the operator into new grids
    Vec3i_cu size(NB_SAMPLES_OCU, NB_SAMPLES_OCU, NB_SAMPLES_ALPHA);
    Grid3_cu<float >* grid_vals  = new Grid3_cu<float >(size, h_vals          );
    Grid3_cu<float2>* grid_grads = new Grid3_cu<float2>(size, (float2*)h_grads);

    // padd operator grids as concatenation won't
    grid_vals-> padd( Vec3i_cu(PADDING, PADDING, PADDING) );
    grid_grads->padd( Vec3i_cu(PADDING, PADDING, PADDING) );
    // record new operator grids
    h_custom_op_vals.push_back( grid_vals );
    h_custom_op_grads.push_back( grid_grads );
    // clean function
    delete[] h_vals;
    delete[] h_grads;
    // return new op id
    updated = false;
    return h_custom_op_vals.size()-1 + NB_PRED_OPS;
}

// -----------------------------------------------------------------------------

Op_id new_op_instance(const std::string &filename)
{
    float*       h_vals  = 0;
    IBL::float2* h_grads = 0;
    int len = (NB_SAMPLES_OCU+2)*(NB_SAMPLES_OCU+2)*(NB_SAMPLES_ALPHA+2);

    h_vals  = new float      [len];
    h_grads = new IBL::float2[len];
    bool s = read_array(h_vals , len, get_cache_dir()+"/"+filename+"_vals.opc"  );
    s = s && read_array(h_grads, len, get_cache_dir()+"/"+filename+"_grads.opc" );

    if (!s)  assert( false );

    // store the operator into new grids
    Vec3i_cu size(NB_SAMPLES_OCU+2, NB_SAMPLES_OCU+2, NB_SAMPLES_ALPHA+2);
    Grid3_cu<float >* grid_vals  = new Grid3_cu<float >(size, h_vals          , PADDING_OFFSET);
    Grid3_cu<float2>* grid_grads = new Grid3_cu<float2>(size, (float2*)h_grads, PADDING_OFFSET);

    // record new operator grids
    h_custom_op_vals.push_back( grid_vals );
    h_custom_op_grads.push_back( grid_grads );
    // clean function
    delete[] h_vals;
    delete[] h_grads;
    // return new op id
    updated = false;
    return h_custom_op_vals.size()-1 + NB_PRED_OPS;
}

// -----------------------------------------------------------------------------

void delete_op_instante(Op_id op_id)
{
    int unsigned i = op_id - NB_PRED_OPS;
    delete h_custom_op_vals[ i ];
    delete h_custom_op_grads[ i ];
    if ( i == h_custom_op_vals.size() - 1 ) {
        // todo : remove all deleted that are in the back ( [000xxX] => [000] )
        h_custom_op_vals.erase(h_custom_op_vals.begin() + i);
        h_custom_op_grads.erase(h_custom_op_grads.begin() + i);
    } else {
        h_custom_op_vals[ i ] = 0;
        h_custom_op_grads[ i ] = 0;
    }
    updated = false;
}

// -----------------------------------------------------------------------------

void make_cache(Op_id op_id, const std::string &filename)
{
    const std::vector<float>&  h_vals  = h_custom_op_vals [op_id - NB_PRED_OPS]->get_vals();
    const std::vector<float2>& h_grads = h_custom_op_grads[op_id - NB_PRED_OPS]->get_vals();

    int len = h_custom_op_vals[op_id - NB_PRED_OPS]->size().product();
    assert( h_custom_op_grads[op_id - NB_PRED_OPS]->size().product() == len);
    if(filename.size() > 0)
    {
        write_array(&(h_vals [0]), len, get_cache_dir()+"/"+filename+"_vals.opc"  );
        write_array(&(h_grads[0]), len, get_cache_dir()+"/"+filename+"_grads.opc" );
    }
}

// -----------------------------------------------------------------------------

void enable_predefined_operator( Op_t op_t, bool on ){
    // TODO: assert() if wrong op types
    h_operators_enabling[ op_t - BINARY_3D_OPERATOR_BEGIN - 1 ] = on;
}

// -----------------------------------------------------------------------------

Op_id get_predefined_op_id(Op_t op_t)
{
    int id_opt = op_t - BINARY_3D_OPERATOR_BEGIN - 1;
    if(id_opt < 0 || !h_operators_enabling[id_opt])
        return -1;

    int res = 0;
    for(int i = 0; i < id_opt; ++i)
        if( h_operators_enabling[i] )
            ++res;

    return res;
}

// -----------------------------------------------------------------------------
//#define SAVE_CUSTOM => Usefull ???? => when load : return list of custom ops Op_ids
void make_cache_env(const std::string &filename)
{
    if(filename.size() == 0)
        return;

    std::string base_name = get_cache_dir()+"/"+filename;

    // save concatenation
    const std::vector<float>&   conc_vals  = grid_operators_values->get_vals();
    const std::vector<float2>&  conc_grads = grid_operators_grads ->get_vals();
    int conc_len = grid_operators_values->size().product();

    write_array(&(conc_vals [0]), conc_len, base_name+"_conc_vals.opc"  );
    write_array(&(conc_grads[0]), conc_len, base_name+"_conc_grads.opc" );

    // save predefined => done through enabling
    // => cf load predifined comment in init_env_fom_cache method
    // save idx
    int idx_len = h_operators_idx_offsets.size();
    write_array(h_operators_idx_offsets.data(), idx_len, base_name+"_offset_idx.opc" );
    // save enabling
    int enab_len = NB_PRED_OPS;
    write_array(h_operators_enabling, enab_len, base_name+"_pred_state.opc" );
    // save infos
    std::ofstream file((base_name+"_infos.opc").c_str(), std::ios_base::out|std::ios_base::trunc);
    if(!file.is_open()){
        std::cerr << "Error exporting file " << filename << std::endl;
        return;
    }

    Vec3i_cu s = grid_operators_values->size();
    file << s.x << " " << s.y << " " << s.z << " " << enab_len << " " << idx_len;
    file.close();
}

// -----------------------------------------------------------------------------

bool init_env_from_cache(const std::string &filename)
{
    std::string base_name = get_cache_dir()+"/"+filename;
    clean_env();
    assert(!binded);

    bool use_cache = true;

    init_profile_bulge(use_cache);
    init_profile_hyperbola(use_cache);
    init_opening_hyperbola(use_cache);
    // get infos
    int conc_s_x, conc_s_y, conc_s_z, enab_len, idx_len;
    std::ifstream file((base_name+"_infos.opc").c_str(), std::ios_base::in);
    if(!file.is_open()){
        std::cerr << "Cache doesn't exists: " << filename << std::endl;
        clean_env();
        return false;
    }
    file >> conc_s_x  >> conc_s_y  >> conc_s_z  >>
            enab_len >> idx_len;
    file.close();
    Vec3i_cu conc_size(conc_s_x, conc_s_y, conc_s_z);
    // restore enabling
    if(enab_len != NB_PRED_OPS){
        clean_env();
        return false;
    }

    Cuda_utils::HA_bool enabled( NB_PRED_OPS );
    if(!read_array(enabled.ptr(), NB_PRED_OPS, base_name+"_pred_state.opc")){
        clean_env();
        return false;
    }
    // test if same config, else return false
    for(int i = 0; i < NB_PRED_OPS; ++i){
        if (enabled[i] != h_operators_enabling[i]){
            clean_env();
            return false;
        }
    }

    // then idx
    h_operators_idx_offsets.resize( idx_len );
    if (!read_array(h_operators_idx_offsets.data(), idx_len, base_name+"_offset_idx.opc" )){
        clean_env();
        return false;
    }
    // then concatenation
    int conc_len = conc_size.product();
    std::vector<float>  conc_vals ( conc_len );
    std::vector<float2> conc_grads( conc_len );
    if (!read_array( &(conc_vals[0]), conc_len, base_name+"_conc_vals.opc" )){
        clean_env();
        return false;
    }

    if (!read_array( &(conc_grads[0]), conc_len, base_name+"_conc_grads.opc" )){
        clean_env();
        return false;
    }
    delete grid_operators_values;
    grid_operators_values = new Grid3_cu<float>( conc_size, &(conc_vals[0]) );
    delete grid_operators_grads;
    grid_operators_grads = new Grid3_cu<float2>( conc_size, &(conc_grads[0]) );
    // then predefined
    load_3d_predefined(); // quicker than conc pred when save and retrieve from conc_grids

    // todo with new env archi
    init_4D_bulge_in_contact(use_cache);
    init_4D_ricci(use_cache);
    init_global_controller();

    Cuda_utils::malloc_d(d_magnitude_3D_bulge, 1);
    Cuda_utils::mem_cpy_htd(d_magnitude_3D_bulge, &h_magnitude_3D_bulge, 1);

    Cuda_utils::malloc_d(d_n_3D_ricci, 1);
    Cuda_utils::mem_cpy_htd(d_n_3D_ricci, &h_n_3D_ricci, 1);

    init_nary_operators();

    allocated = true;
    // allocate on gpu without concatenate
    allocate_and_copy_3D_array(grid_operators_values, d_operators_values);
    allocate_and_copy_3D_array(grid_operators_grads, d_operators_grads);
    // upload idx offsets
    std::vector< int4 > indices(h_operators_idx_offsets.size());
    for(unsigned i = 0; i < indices.size(); ++i)
        indices[i] = h_operators_idx_offsets[i].to_int4();
    d_operators_idx_offsets.malloc( indices.size() );
    d_operators_idx_offsets.copy_from( indices );
    // upload pred ids
    std::vector<int> pred_id(NB_PRED_OPS, -1);
    int id = 0;
    for(int i = 0; i < NB_PRED_OPS; ++i){
        if(h_operators_enabling[i]){
            pred_id[i] = id;
            ++id;
        }
    }
    d_operators_id.malloc( pred_id.size() );
    d_operators_id.copy_from( pred_id );

    bind();
    return true;
}



void set_global_ctrl_shape(const IBL::Ctrl_setup& shape)
{
    CUDA_SAFE_CALL(cudaUnbindTexture(global_controller_tex));

    IBL::float2* controller = 0;
    globale_ctrl_shape = shape;
    IBL::gen_controller(NB_SAMPLES, globale_ctrl_shape, controller);

    int data_size = NB_SAMPLES * sizeof(float2);
    CUDA_SAFE_CALL(cudaMemcpyToArray(d_global_controller, 0, 0, (float2*)controller, data_size, cudaMemcpyHostToDevice));

    delete[] controller;

    CUDA_SAFE_CALL(cudaBindTextureToArray(global_controller_tex, d_global_controller));
}

// -----------------------------------------------------------------------------

/// Magnitude of the bulge in contact :
/// Value is to be between [0.0, 0.9] outside the operator doesn't
/// behave well.
/// @warning the operator must be updated to take into acount the new value
/// with update_3D_bulge()
void set_bulge_magnitude(float mag)
{
    CUDA_SAFE_CALL(cudaUnbindTexture(magnitude_3D_bulge_tex));

    h_magnitude_3D_bulge = mag;
    Cuda_utils::mem_cpy_htd(d_magnitude_3D_bulge, &h_magnitude_3D_bulge, 1);

    CUDA_SAFE_CALL(cudaBindTexture(0, magnitude_3D_bulge_tex, d_magnitude_3D_bulge, sizeof(float)));
}

// -----------------------------------------------------------------------------

/// N of the ricci operator.
/// @warning the operator must be updated to take into acount the new value
/// with update_3D_ricci()
void set_ricci_n(float N)
{
    CUDA_SAFE_CALL(cudaUnbindTexture(n_3D_ricci_tex));

    h_n_3D_ricci = N;
    Cuda_utils::mem_cpy_htd(d_n_3D_ricci, &h_n_3D_ricci, 1);

    CUDA_SAFE_CALL(cudaBindTexture(0, n_3D_ricci_tex, d_n_3D_ricci, sizeof(float)));
}

}// END PRECOMPUTED FUNCTIONS ==================================================

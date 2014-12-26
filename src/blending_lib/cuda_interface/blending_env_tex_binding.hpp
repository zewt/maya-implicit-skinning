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
#ifndef BLENDING_ENV_TEX_BINDING_HPP__
#define BLENDING_ENV_TEX_BINDING_HPP__

#include "blending_env.hpp"
#include "blending_lib/generator.hpp"
#include "blending_lib/controller.hpp"

/**
  Please refer to the documentation of blending_env_tex.hpp for more details
  on how to use this header

  @warning This file may only be included once in the project
  (i.e. in a single .cu), and ONLY after the inclusion of the header
  blending_env_tex.hpp

  @see blending_env_tex.hpp
*/


#if !defined(BLENDING_ENV_TEX_HPP__)
#error "You must include blending_env_tex.hpp before the inclusion of 'blending_env_tex_binding.hpp'"
#endif

// =============================================================================
namespace Blending_env{
// =============================================================================

bool binded = false;

// -----------------------------------------------------------------------------

void bind()  { bind_local();   binded = true;  }
void unbind(){ unbind_local(); binded = false; }

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

}// END Blending_env ===========================================================

#endif // BLENDING_ENV_TEX_BINDING_HPP__

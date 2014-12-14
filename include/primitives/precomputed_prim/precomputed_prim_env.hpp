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
#ifndef PRECOMPUTED_PRIM_ARRAY_HPP__
#define PRECOMPUTED_PRIM_ARRAY_HPP__

#include "precomputed_prim_constants.hpp"
#include "cuda_utils.hpp"
#include "transfo.hpp"

class Bone;
// FIXME: we should pad the 3d grid to avoid the lerp artifacts at borders
// TODO: we should be able to store any kind of implicit prim (template or class polyphormism)
/** @namespace Precomputed_env
    @brief Environment storing 3D grids representing implicit primitives

    Precomputed_Env provides a way to store in Cuda textures 3d grids and
    fetch them with trilinear interpolation

    How to upload one primitive and transform it:
    @code
    int id = new_instance();
    // init the 3d grid potential and gradient with a implicit bone
    init_instance(id, bone);
    // Apply a global transformation to the 3d grid
    set_transform(id, tr);
    set_transform(id, tr);
    // To take into account the transformations
    update_device_transformations();
    @endcode
*/
// =============================================================================
namespace Precomputed_env{
// =============================================================================

/// In Cuda textures has a limited size of 2048^3 (considering a GPU with
/// compute capability 2.0). so for one texture we stack on the x, y and z axis
/// multiples grids. 'd_block' is a block made of multiples grids.
extern Cuda_utils::Device::CuArray<float4>  d_block;

/// Transformation of a 3d point to the texture coordinates of 'tex_grids'.
/// d_anim_transform[inst_id] = tex_grid_transfo
extern Cuda_utils::Device::Array<Transfo> d_anim_transform;

extern Cuda_utils::Device::Array<Transfo> d_grad_transform;

/// This array is used to look up the 'd_block' array in order to find the grid
/// attached to an instance identifier. d_offset[id].x/y/z coordinates represent
/// the indices inside the block 'd_block' for the grid of identifier 'id'.
/// @code
/// int x = d_offset[id].x * GRID_RES;
/// int y = d_offset[id].y * GRID_RES;
/// int z = d_offset[id].z * GRID_RES;
/// int i = x + y * MAX_TEX_LENGTH + z * MAX_TEX_LENGTH * MAX_TEX_LENGTH
/// d_block[i] // first element of the grid of indentifier 'id'
/// @endcode
/// GRID_SIZE defines the x, y and z length of the grid
/// @see Precomputed_Env::tex_grids
extern Cuda_utils::Device::Array<int4> d_offset;

/// True when the arrays are binded to textures
extern bool binded;

// -----------------------------------------------------------------------------

void clean_env();

int new_instance();

void delete_instance(int inst_id);

void reset_instance(int inst_id);

void init_instance(int inst_id, const Bone* bone);

/// get the transformation t set by 'set_transform()'
const Transfo& get_user_transform(int inst_id);

/// In order to animate the precomputed primitives one as to set the
/// transformations applied to each primitive.
/// @warning One must call update_device_transformations() setting all the
/// instances transformation
/// @see update_device_transformations()
void set_transform(int inst_id, const Transfo& transfo);

/// Copy the host transformations set by set_transform() to texture
/// @see set_transform()
void update_device_transformations();

/// @name binding functions
/// implemented with precomputed_prim_env_binding.hpp.
/// @see precomputed_prim_env_binding.hpp
/// @{
void bind();
void unbind();
/// @}


}// END PRECOMPUTED_ENV NAMESPACE ==============================================

#endif // PRECOMPUTED_PRIM_ARRAY_HPP__

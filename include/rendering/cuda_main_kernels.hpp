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
#ifndef CUDA_MAIN_KERNELS_HPP__
#define CUDA_MAIN_KERNELS_HPP__

// -----------------------------------------------------------------------------

#include <vector>

// -----------------------------------------------------------------------------

#include "bbox.hpp"
#include "raytracing_context.hpp"
#include "cuda_utils.hpp"
#include "scene_enum.hpp"
#include "blending_env_type.hpp"

// -----------------------------------------------------------------------------

/**
 * @file cuda_main_kernels.cu
 * @brief holds the main cuda kernels of the project with access to the main
 * cuda textures in the project
 *
 * In order to get maximum performances we use cuda textures which have the
 * advantage of using the GPU cache. However there is catch. A texture reference
 * must be a global and its visibility in the project is
 * limited to its translation unit (the'.cu' file its declared within).
 *
 * you can declare texture references with the same name in different '.cu'
 * it won't change the fact you need to bind <b>each</b> textures
 * to a cuda array (even if its the same array).
 *
 * We use a lot of textures and want to avoid binding/unbinding each time we
 * call a kernel. To this end we define a single file 'cuda_main_kernels.cu'
 * where textures will be binded once at start up.
 * only critical kernels called very often needs to be in this file. Other
 * kernels can be defined in others '.cu' but must bind/unbind the texture
 * at each kernel call.
*/

// -----------------------------------------------------------------------------

/// Initialize device memory and textures
/// @warning must be called first before any other cuda calls
void init_cuda(const std::vector<Blending_env::Op_t>& op);

/**
 * @namespace Raytracing
 * @brief functions related to raytracing and cuda kernels associated
*/
// =============================================================================
namespace Raytracing {
// =============================================================================

/// Update the current bounding box to raytrace
void update_bbox(const BBox_cu& bbox);

/// Do we partially evaluate the tree ?
/// @see set_bone_to_trace()
void set_partial_tree_eval(bool s);

/// Sets the eight first bone to be raytraced from the vector
void set_bone_to_trace(const std::vector<int>& bone_ids);

void trace( Context& ctx );

void trace_skinning_skeleton(const Context& ctx);

void set_scene_type(Scene_Type type);

}// End Raytracing namespace ===================================================


/// Fetch a controller value from texture given its instance id.
__global__
void get_controller_values(Cuda_utils::Device::Array<float2> out_vals, int inst_id);

// =============================================================================



#endif // CUDA_MAIN_KERNELS_HPP__

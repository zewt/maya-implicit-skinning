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
#ifndef ANIMESH_KERS_COLORS_HPP__
#define ANIMESH_KERS_COLORS_HPP__

#include "mesh.hpp"
#include "animesh_enum.hpp"
#include "bbox.hpp"
#include "color.hpp"
#include "cuda_utils.hpp"

/**
 * @name Animesh_colors
 * @brief Regroup kernels related to Animesh and color painting
 *
 * @see Animesh
*/
// =============================================================================
namespace Animesh_colors{
// =============================================================================

/// Set the color of each vertex depending on which cluster they belong to
__global__
void cluster_colors_kernel(float4* d_colors,
                           const Mesh::Packed_data* d_map,
                           Cuda_utils::Device::Array<int> d_vertices_nearest_bones);

/// Set the color of each vertex depending on the ssd interpolation factor
/// between SSD and implicit skinning
/// Red for full SSD and yellow for full implicit skinning
__global__
void ssd_interpolation_colors_kernel(float4* d_colors,
                                     const Mesh::Packed_data* d_map,
                                     Cuda_utils::Device::Array<float> d_ssd_interpolation_factor);

/// Set the color of each vertex depending on the smoothing factor
/// Red for full smoothing and yellow for no smothing
__global__
void smoothing_colors_kernel(float4* d_colors,
                             const Mesh::Packed_data* d_map,
                             Cuda_utils::Device::Array<float> d_smoothing_factors);

__global__
void nearest_joint_colors_kernel(float4* d_colors,
                                 const Mesh::Packed_data* d_map,
                                 Cuda_utils::Device::Array<int> d_vertices_nearest_joint);

/// Set the color of each vertex depending on the ssd weights
__global__
void normal_colors_kernel(float4* d_colors,
                          const Mesh::Packed_data* d_map,
                          const Vec3_cu* d_normals,
                          int size);


/// Set the color of each vertex depending on their base potential
/// Smooth transition is done from red to green and then blue where
/// red is 0, green is 0.5 and blue 1; Nan numbers are white
__global__
void base_potential_colors_kernel(float4* d_colors,
                                  const Mesh::Packed_data* d_map,
                                  Cuda_utils::Device::Array<float> d_base_potential);

/// set the color of each vertex depending on their current gradient potential
__global__
void gradient_potential_colors_kernel(float4* d_colors,
                                      const Mesh::Packed_data* d_map,
                                      Vec3_cu* vertices,
                                      int n);

/// Set a uniform color
__global__
void user_defined_colors_kernel(float4* d_colors,
                                const Mesh::Packed_data* d_map,
                                float4 color,
                                int n);

__global__
void vert_state_colors_kernel(float4* d_colors,
                              const Mesh::Packed_data* d_map,
                              Cuda_utils::Device::Array<EAnimesh::Vert_state> d_vertices_state,
                              Cuda_utils::Device::Array<float4> d_vertices_states_color);

/// Color given mean value coordinates
__global__
void mvc_colors_kernels(float4* d_colors,
                        const Mesh::Packed_data* d_map,
                        const int* edge_list,
                        const int* edge_list_offsets,
                        const float* edge_mvc,
                        int nb_verts);

/// Color according to the ssd weights associated to the if joint "joint_id"
__global__
void ssd_weights_colors_kernel(float4* d_colors,
                               const Mesh::Packed_data* d_map,
                               int joint_id,
                               Cuda_utils::Device::Array<int> d_jpv,
                               Cuda_utils::Device::Array<float> d_weights,
                               Cuda_utils::Device::Array<int> d_joints );




}// ============================================================================

#endif // ANIMESH_KERS_COLORS_HPP__

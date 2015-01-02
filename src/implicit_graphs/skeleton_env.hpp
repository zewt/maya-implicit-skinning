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
#ifndef SKELETON_ENV_HPP__
#define SKELETON_ENV_HPP__

#include <vector>

#include "skeleton_env_type.hpp"
#include "bone.hpp"
#include "bone_tex.hpp"
#include "cuda_utils.hpp"
#include "bone_tex.hpp"


/** @namespace Skeleton_env
    @brief provides means for storing and fetching bones and their
    connexity into cuda textures.

    Here bones connexity is expressed in terms of clusters.

    A cluster is a set of bone which belongs to the same level and have the same
    bone parent :
    @code
             +
            /              // Here we have 3 cluster.
           /               // there is 2 cluster composed of only one bone
    +-----+---+-----+      // and one cluster of three bones.
           \
            \
             +
    @endcode

    @warning Here the index of a bone is different from the index of a bone in
    Bone class. In device mem bones of the same cluster are contigus.
    DBone_id is used to store bones in Skeleton_env while Bone::Id is left to
    the user responbility to be set

    @warning For the moment only a single instance of a skeleton can be stored
 */
// =============================================================================
namespace Skeleton_env {
// =============================================================================

extern Bone_tex* hd_bone_arrays;

/// Concatenated blendind list for every skeletons
extern Cuda_utils::HD_Array<Cluster_cu> hd_blending_list;

/// Concatenated datas os the blending list
extern Cuda_utils::HD_Array<Cluster_data> hd_cluster_data;

/// Concatenated blendind list for every skeletons in each grid cell.
/// Each cell store a single blending list. Cells are linearly arranged in
/// memory as well as the blending list.
extern Cuda_utils::HD_Array<Cluster_cu> hd_grid_blending_list;

/// hd_grid[ hd_offset[Skel_id].grid_data + cell_idx] == offset in hd_grid_blending_list or -1 if empty cell
extern Cuda_utils::HD_Array<int> hd_grid;

/// Bbox of each skeleton's grid.
/// (hd_grid_bbox[i*2 + 0], hd_grid_bbox[i*2 + 0]) == ith_skel_bbox
extern Cuda_utils::HD_Array<float4> hd_grid_bbox;

/// d_offset[bone_id] == offset in lists
extern Cuda_utils::HD_Array<Offset>  hd_offset;

// -----------------------------------------------------------------------------

/// Erase environment
void clean_env();

/// Allocate memory.
/// To copy data of the skeleton on the Device use update_bones_device_mem()
/// and update_joints_device_mem()
void init_env();

/// Set resolution for the acceleration structure grid
void set_grid_res(Skel_id id, int res);

/// Create a new skeleton instance
Skel_id new_skel_instance(const std::vector<const Bone*>& bones,
                          const std::vector<int>& parents);

void delete_skel_instance(Skel_id i);

/// Update the bone data ( type length etc.) of the skeleton in device memory
void update_bones_data(Skel_id i, const std::vector<const Bone*>& bones);

/// Update the joints data (type, controller id, bulge strength)
/// in device memory.
/// @note The type of blending between a bone and its parent is defined by the
/// parent bone.
void update_joints_data(Skel_id i, const std::vector<Joint_data>& joints);

DBone_id bone_hidx_to_didx(Skel_id skel_id, Bone::Id bone_hidx);
Bone::Id bone_didx_to_hidx(Skel_id skel_id, DBone_id bone_didx);

/// @defgroup bindings
/// implemented in skeleton_env_tex_binding.hpp
/// @{
void bind();
void unbind();
/// @}

}// End Skeleton_env ===========================================================

#endif // SKELETON_ENV_HPP__

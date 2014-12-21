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
#ifndef SKELETON_ENV_TEX_HPP__
#define SKELETON_ENV_TEX_HPP__

#include "skeleton_env.hpp"

/** This file contains functions that stores the current skeleton into
    CUDA textures.
    @warning It MUST be included in CUDA main program file.
*/

/**
 * To see how to use these textures take a look to compute_potential()
 * in skeleton_env_evaluator.hpp
*/

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// Cluster list for the whole skeleton.
/// @li x : Nb bone
/// @li y : first bone id
/// @li z : blending type
/// @li w : ctrl_id
/// @remarks Its more convenient to use 'struct Skeleton_env::Cluster_cu'
/// than the int4.
/// @see Skeleton_env::Cluster_cu
texture<int4, 1, cudaReadModeElementType> tex_blending_list;

/// Grid cells concatenated blending list
/// Cluster_cu (as describded with 'tex_blending_list')
/// @see Skeleton_env::Cluster_cu tex_blending_list
texture<int4, 1, cudaReadModeElementType> tex_grid_list;

/// Concatenated grid cells stored linearly
texture<int, 1, cudaReadModeElementType> tex_grid;

/// every grids bbox and resolution.
/// (bbox.pmin.x, bbox.pmin.y, bbox.pmin.z) == tex_grid_bbox[id_skel*2+0]{x,y,z}
/// res == (int)tex_grid_bbox[id_skel*2+0]{w}
/// (bbox.pmax.x, bbox.pmax.y, bbox.pmax.z) == tex_grid_bbox[id_skel*2+1]{x,y,z}
/// dummy float == tex_grid_bbox[id_skel*2+1]{w}
texture<float4, 1, cudaReadModeElementType> tex_grid_bbox;

// TODO: store nb_pairs and singletons in offset
/// Offset to access the blending list or grid according to the skeleton
/// instance tex_offset[skel_id] = offset
/// @li x : offset to access tex_blending_list
/// @li y : tex_blending_list tex_grid_list
/// @remarks Its more convenient to use 'struct Skeleton_env::Offset'
/// than the int4.
/// @see Skeleton_env::Offset
texture<int2, 1, cudaReadModeElementType> tex_offset;

/// At each element of 'tex_blending_list' corresponds a bulge strength in
/// tex_bulge_strength
texture<float, 1, cudaReadModeElementType> tex_bulge_strength;

// -----------------------------------------------------------------------------
/// @name Per bones data
// -----------------------------------------------------------------------------

/// Bone's types indicates which bone texture has to be fetched
texture<int, 1, cudaReadModeElementType> tex_bone_type;

/// Bones of same cluster are stored consecutively
texture<float4, 1, cudaReadModeElementType> tex_bone;
texture<int   , 1, cudaReadModeElementType> tex_bone_hrbf;
texture<int   , 1, cudaReadModeElementType> tex_bone_precomputed;

// -----------------------------------------------------------------------------

static void bind_local();
static void unbind_local();

// -----------------------------------------------------------------------------
/// @name Blending list (with acceleration structure)
// -----------------------------------------------------------------------------

/// @param id : skeleton identifier
/// @param pos : world position the blending list will be evaluated
/// @return first element of the blending list contains in the grid cell
/// at pos. TO evaluate the blending list use fetch_grid_blending_list()
IF_CUDA_DEVICE_HOST static inline
Cluster_id fetch_grid_blending_list_offset(Skel_id id, const Vec3_cu& pos);

/// Blending list of every skeletons for every grid's cells
IF_CUDA_DEVICE_HOST static inline
Cluster_cu fetch_grid_blending_list(Cluster_id i);

// -----------------------------------------------------------------------------
/// @name Blending list (no acceleration structure)
// -----------------------------------------------------------------------------

/// Fetch the offset needed to use fetch_blending_list() given a specific
/// skeleton instance.
__device__ static inline
Cluster_id fetch_blending_list_offset(Skel_id id);

/// List of cluster pairs. The list is composed in two parts pairs to be blended
/// with a specific operators and singletons to be blended with an n-ary operator
/// altogether with the blended pairs (we use max for the n-ary).
/// The first element does not specify the blending type and controller id but :
/// z == nb_pairs and w = nb_singletons after the pairs.
/// @param i : identifier of the cluster plus fetch_blending_list_offset()
__device__ static inline
Cluster_cu fetch_blending_list(Cluster_id i);

// -----------------------------------------------------------------------------
/// @name Blending list datas (bones and blending operators)
// -----------------------------------------------------------------------------

/// Read data of bone no. i.
/// @warning the index i is note the same as the index in the Skeleton class
/// to get the correspondance use the function get_idx_device_bone()
/// @see get_idx_device_bone()
__device__ static inline
Bone_cu fetch_bone_cu(DBone_id i);

/// Read data of a bone of type hrbf
/// @warning User must ensure that the bone i is of the right type with
/// fetch_bone_type() otherwise returned value is undefined
__device__ static inline
HermiteRBF fetch_bone_hrbf(DBone_id i);

/// Read data of a bone of type precomputed
/// @warning User must ensure that the bone i is of the right type with
/// fetch_bone_type() otherwise returned value is undefined
__device__ static inline
Precomputed_prim fetch_bone_precomputed(DBone_id i);

/// @return the bone type defined in the enum of Bone_type namespace
/// @see Bone_type
__device__ static inline
EBone::Bone_t fetch_bone_type(DBone_id bone_id);

/// Fetch a bone and evaluate its potential.
/// @param bone_id the bone id
/// @param gf the gradient at point x
/// @return Potential at point x
__device__ static inline
float fetch_and_eval_bone(DBone_id bone_id, Vec3_cu& gf, const Point_cu& x);

/// Fetch a blending operator and blend the potential
/// @param gf the blended gradient
/// @param type The blending type
/// @param ctrl_id the controller id for the blending op if any.
/// @param f1 First potential value to blend
/// @param f2 Second potential value to blend
/// @param gf1 First gradient to blend
/// @param gf2 Second gradient to blend
/// @return the blended potential
__device__ static inline
float fetch_binop_and_blend(Vec3_cu& gf,
                            EJoint::Joint_t type,
                            Blending_env::Ctrl_id ctrl_id,
                            Cluster_id clus_id,
                            float f1, float f2,
                            const Vec3_cu& gf1, const Vec3_cu& gf2);

}// END SKELETON_TEXTURES NAMESPACE ============================================


#include "skeleton_env_tex.inl"

#endif // SKELETON_ENV_TEX_HPP__

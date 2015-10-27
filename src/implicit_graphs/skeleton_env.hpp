#ifndef SKELETON_ENV_HPP__
#define SKELETON_ENV_HPP__

#include <vector>
#include <map>

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
                          const std::map<Bone::Id, Bone::Id>& parents,
                          int grid_res=-1);

void delete_skel_instance(Skel_id i);

/// Update the bone data ( type length etc.) of the skeleton in device memory
void update_bones_data(Skel_id i);

/// Update the joints data (type, controller id, bulge strength)
/// in device memory.
/// @note The type of blending between a bone and its parent is defined by the
/// parent bone.
void update_joints_data(Skel_id i, const std::map<Bone::Id, Joint_data>& joints);

DBone_id bone_hidx_to_didx(Skel_id skel_id, Bone::Id bone_hidx);
Bone::Id bone_didx_to_hidx(Skel_id skel_id, DBone_id bone_didx);

/// Cluster list for the whole skeleton.
/// @li x : Nb bone
/// @li y : first bone id
/// @li z : blending type
/// @li w : ctrl_id
/// @remarks Its more convenient to use 'struct Skeleton_env::Cluster_cu'
/// than the int4.
/// @see Skeleton_env::Cluster_cu
extern texture<int4, 1, cudaReadModeElementType> tex_blending_list;

/// Grid cells concatenated blending list
/// Cluster_cu (as describded with 'tex_blending_list')
/// @see Skeleton_env::Cluster_cu tex_blending_list
extern texture<int4, 1, cudaReadModeElementType> tex_grid_list;

/// Concatenated grid cells stored linearly
extern texture<int, 1, cudaReadModeElementType> tex_grid;

/// every grids bbox and resolution.
/// (bbox.pmin.x, bbox.pmin.y, bbox.pmin.z) == tex_grid_bbox[id_skel*2+0]{x,y,z}
/// res == (int)tex_grid_bbox[id_skel*2+0]{w}
/// (bbox.pmax.x, bbox.pmax.y, bbox.pmax.z) == tex_grid_bbox[id_skel*2+1]{x,y,z}
/// dummy float == tex_grid_bbox[id_skel*2+1]{w}
extern texture<float4, 1, cudaReadModeElementType> tex_grid_bbox;

// TODO: store nb_pairs and singletons in offset
/// Offset to access the blending list or grid according to the skeleton
/// instance tex_offset[skel_id] = offset
/// @li x : offset to access tex_blending_list
/// @li y : tex_blending_list tex_grid_list
/// @remarks Its more convenient to use 'struct Skeleton_env::Offset'
/// than the int4.
/// @see Skeleton_env::Offset
extern texture<int2, 1, cudaReadModeElementType> tex_offset;

/// At each element of 'tex_blending_list' corresponds a bulge strength in
/// tex_bulge_strength
extern texture<float, 1, cudaReadModeElementType> tex_bulge_strength;

// -----------------------------------------------------------------------------
/// @name Per bones data
// -----------------------------------------------------------------------------

/// Bone's types indicates which bone texture has to be fetched
extern texture<int, 1, cudaReadModeElementType> tex_bone_type;

/// Bones of same cluster are stored consecutively
extern texture<int   , 1, cudaReadModeElementType> tex_bone_hrbf;
extern texture<int   , 1, cudaReadModeElementType> tex_bone_precomputed;

// -----------------------------------------------------------------------------

void bind();
void unbind();

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

}// End Skeleton_env ===========================================================

#include "skeleton_env.inl"

#endif // SKELETON_ENV_HPP__

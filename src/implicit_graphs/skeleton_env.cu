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
#include "skeleton_env.hpp"
#include "skeleton.hpp"

#include "std_utils.hpp"
#include "grid.hpp"
#include "tree_cu.hpp"
#include "tree.hpp"
#include <list>
#include <deque>
#include <map>
#include <set>

namespace { __device__ void fix_debug() { } }

// =============================================================================
namespace Skeleton_env {
// =============================================================================

bool allocated = false;
bool binded;

// -----------------------------------------------------------------------------
/// @name GPU friendly datas
// -----------------------------------------------------------------------------

Bone_tex* hd_bone_arrays = 0;

/// Concatenated blendind list for every skeletons
Cuda_utils::HD_Array<Cluster_cu> hd_blending_list;

Cuda_utils::HD_Array<Cluster_data> hd_cluster_data;

texture<int4, 1, cudaReadModeElementType> tex_blending_list;
texture<int4, 1, cudaReadModeElementType> tex_grid_list;
texture<int, 1, cudaReadModeElementType> tex_grid;
texture<float4, 1, cudaReadModeElementType> tex_grid_bbox;
texture<int2, 1, cudaReadModeElementType> tex_offset;
texture<float, 1, cudaReadModeElementType> tex_bulge_strength;
texture<int, 1, cudaReadModeElementType> tex_bone_type;
texture<int   , 1, cudaReadModeElementType> tex_bone_hrbf;
texture<int   , 1, cudaReadModeElementType> tex_bone_precomputed;

/// Concatenated blendind list for every skeletons in each grid cell not empty.
/// Each cell store a single blending list.
/**
 * @code
 *  |-list0-|-list1-|-list2-| |-list0-|-list1-|
 *  *-----------------------* *---------------*
 *            skel0                 skel1
 * @endcode
*/

class SkeletonEnv
{
public:
    SkeletonEnv();
    ~SkeletonEnv();

    /// Skeletons instance, a skeleton must be a tree with one connex componant.
    Tree *h_tree;

    /// Acceleration structure for the different skeletons in the environment

    Grid *h_grid;

    Tree_cu *h_tree_cu_instance;
};

std::deque<SkeletonEnv *> h_envs;

Cuda_utils::HD_Array<Cluster_cu> hd_grid_blending_list;

/// Concatenated datas corresponding to clusters listed hd_grid_blending_list
Cuda_utils::HD_Array<Cluster_data> hd_grid_data;
/// Table of indirection which maps grid cells to blending list.
/// hd_grid[ hd_offset[Skel_id].grid_data + cell_idx] == offset in hd_grid_blending_list or -1 if empty cell
Cuda_utils::HD_Array<int> hd_grid;

Cuda_utils::HD_Array<float4> hd_grid_bbox;

/// d_offset[Skel_id] == offset in lists or grid
Cuda_utils::HD_Array<Offset>  hd_offset; // maybe we should cut in half offset and allocate it in advance at each new skeleton instances

// -----------------------------------------------------------------------------
/// @name CPU friendly datas
// -----------------------------------------------------------------------------

/// user idx to device bone idx
std::map<Hbone_id, DBone_id> _hidx_to_didx;
/// device bone idx to user idx
std::map<DBone_id, Hbone_id> _didx_to_hidx;



SkeletonEnv::SkeletonEnv()
{
    h_tree = NULL;
    h_tree_cu_instance = NULL;
    h_grid = NULL;
}

SkeletonEnv::~SkeletonEnv()
{
    delete h_tree;
    delete h_tree_cu_instance;
    delete h_grid;
}


void bind()
{
    using namespace Cuda_utils;
    
    binded = true;
    // Initialize texture and bind them to the arrays
    if(hd_bone_arrays != 0)
    {
        hd_bone_arrays->hd_bone_hrbf.       device_array().bind_tex(tex_bone_hrbf       );
        hd_bone_arrays->hd_bone_precomputed.device_array().bind_tex(tex_bone_precomputed);
        hd_bone_arrays->hd_bone_types.      device_array().bind_tex(tex_bone_type       );
    }

    hd_cluster_data      .device_array().bind_tex( tex_bulge_strength  );
    hd_offset            .device_array().bind_tex( tex_offset          );
    hd_blending_list     .device_array().bind_tex( tex_blending_list   );
    hd_grid              .device_array().bind_tex( tex_grid            );
    hd_grid_blending_list.device_array().bind_tex( tex_grid_list       );
    hd_grid_bbox         .device_array().bind_tex( tex_grid_bbox       );
}

// -----------------------------------------------------------------------------

void unbind()
{
    binded = false;
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_bone_hrbf)        );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_bone_precomputed) );

    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_blending_list)    );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_bone_type)        );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_bulge_strength)   );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_offset)           );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_grid)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_grid_list)        );
    CUDA_SAFE_CALL( cudaUnbindTexture(&tex_grid_bbox)        );
}

// -----------------------------------------------------------------------------

// =============================================================================

/// @param sid : skeleton identifier
/// @param cell_id : linear index of the 3d cell we want to extract the blending
/// list
/// @param blist : described the sub-skeleton in the
void cell_to_blending_list(Skel_id sid,
                           int cell_id,
                           std::vector<Cluster>& blist)
{
    // Note: if in each cell the list of bones is order from root to leaf
    // then blending list will be also ordered root to leaf
    const Grid*    g    = h_envs[sid]->h_grid;
    const Tree_cu* tree = h_envs[sid]->h_tree_cu_instance;
    const std::list<Bone::Id> &bones_in_cell = g->_grid_cells[cell_id];

    std::vector<bool> cluster_done(tree->_clusters.size(), false);

    for(Bone::Id bone_id: bones_in_cell)
    {
        DBone_id dbone = tree->hidx_to_didx(bone_id);

        Cluster_id clus_id = tree->bone_to_cluster( dbone );
        if( cluster_done[clus_id.id()] ) continue;

        tree->add_cluster( clus_id, blist );

        cluster_done[clus_id.id()] = true;
    }
}

// -----------------------------------------------------------------------------

/// Fill device array : hd_grid_blending_list; hd_offset (only grid_data field);
/// hd_grid; hd_grid_data
static void update_device_grid()
{
#if 1
    assert( !binded );
    hd_grid.fill( -1 );

    // Look up every grids and compute the cells blending list.
    // update offset to access blending list as well
    int offset = 0;
    int grid_offset = 0;
    int off_bone = 0;

    for(unsigned grid_id = 0; grid_id < h_envs.size(); ++grid_id)
    {
        if(h_envs[grid_id] == NULL)
            continue;

        const Grid* grid = h_envs[grid_id]->h_grid;
        const Tree_cu *tree = h_envs[grid_id]->h_tree_cu_instance;

//        grid->build_grid(); should be already done
//        ((Grid *)grid)->build_grid(); // (but isn't always)

        // Get the blending list for each cell, and the total number of resulting clusters.
        std::map<int, std::vector<Cluster> > blist_per_cell;
        int total_size = 0;
        for(int cell_idx: grid->_filled_cells) {
            cell_to_blending_list(grid_id, cell_idx, blist_per_cell[cell_idx]);
            total_size += blist_per_cell[cell_idx].size();
        }

        // Allocate space for these clusters.
        hd_grid_blending_list.realloc(offset + total_size);
        hd_grid_data.realloc(offset + total_size);

        for(int cell_idx: grid->_filled_cells)
        {
            std::vector<Cluster> &blist = blist_per_cell.at(cell_idx);
            if(blist.size() > 0)
                blist[0].datas._blend_type = (EJoint::Joint_t)(tree->_blending_list.size()/2);

            hd_grid[grid_offset + cell_idx] = offset;

            for(const Cluster &c: blist)
            {
                // Convert cluster to cluster_cu and offset bones id to match the concateneted representation
                Cluster_cu clus(c);

                clus.first_bone += off_bone;

                hd_grid_blending_list[offset] = clus;
                hd_grid_data         [offset]._bulge_strength = c.datas._bulge_strength;
                offset++;
            }
        }

        hd_offset[grid_id].grid_data = grid_offset;
        const int res = grid->res();
        grid_offset += res*res*res;

        // Update grid bbox and resolution
        BBox_cu bb = grid->bbox();
        hd_grid_bbox[grid_id*2 + 0] = bb.pmin.to_float4();
        hd_grid_bbox[grid_id*2 + 1] = bb.pmax.to_float4();
        hd_grid_bbox[grid_id*2 + 0].w = (float)res;

        off_bone += h_envs[grid_id]->h_tree_cu_instance->_bone_aranged.size();
    }

    hd_offset.update_device_mem(); // This is also done in update_device_tree maybe we can factorize
    hd_grid.update_device_mem();
    hd_grid_blending_list.update_device_mem();
    hd_grid_data.update_device_mem();
    hd_grid_bbox.update_device_mem();
#endif
}

// -----------------------------------------------------------------------------

/// Fill device array : hd_bone_types; hd_bone_hrbf;
/// hd_bone_precomputed; hd_bulge_strength;
static void fill_separated_bone_types(const std::vector<const Bone*>& generic_bones)
{
    assert( !binded   );
    assert( allocated );

    const int nb_bones = generic_bones.size();
    hd_bone_arrays->resize( nb_bones );

    // For each bone, store the type, and the bone's HRBF and primitive ID.  We can store
    // the IDs even if the bone is in a different mode.
    for(int i = 0; i < nb_bones; i++)
    {
        const Bone* b = generic_bones[i];
        hd_bone_arrays->hd_bone_hrbf[i] = b->get_hrbf();
        hd_bone_arrays->hd_bone_precomputed[i] = b->get_primitive();
        hd_bone_arrays->hd_bone_types[i] = b->get_type();
    }
    // Upload every arrays to GPU
    hd_bone_arrays->update_device_mem();
}

// -----------------------------------------------------------------------------


/// Fill device array : hd_blending_list; hd_offset (only list_data field);
/// h_generic_bones; _hidx_to_didx; _didx_to_hidx;
static void update_device_tree(std::vector<const Bone*> &h_generic_bones)
{
    assert( !binded );
    // Convert host layout to the GPU friendly layout
    // And compute some array sizes.

    int s_blend_list = 0; // Total size of all blending lists
    for(unsigned i = 0; i < h_envs.size(); ++i)
    {
        if(h_envs[i] == NULL)
            continue;

        // Convert tree to GPU layout
        delete h_envs[i]->h_tree_cu_instance;
        h_envs[i]->h_tree_cu_instance = new Tree_cu( h_envs[i]->h_tree );
        s_blend_list += h_envs[i]->h_tree_cu_instance->_blending_list.size();
    }

    // Now we can allocate memory
    hd_offset.malloc( h_envs.size() );

    hd_blending_list.malloc( s_blend_list );
    hd_cluster_data. malloc( s_blend_list );

    _hidx_to_didx.clear();
    _didx_to_hidx.clear();

    // Concatenate bones and blending list.
    // Note that the bone identifiers in the new blending list must
    // be changed to match the list of concatenated bones

    int off_bone  = 0; // Offset to store bones in h_bone_device
    int off_blist = 0; // Offset to store blending list in
    for(unsigned t = 0; t < h_envs.size(); ++t)
    {
        if(h_envs[t] == NULL)
            continue;

        const Tree_cu* tree_cu = h_envs[t]->h_tree_cu_instance;

        for(unsigned i = 0; i < tree_cu->_bone_aranged.size(); ++i){
            DBone_id new_didx = DBone_id(i) + off_bone;
            Hbone_id hidx(t, tree_cu->get_id_bone_aranged( i ) );
            h_generic_bones.push_back(tree_cu->_bone_aranged[i]);
            // Build correspondance between device/host index for the
            // concatenated bones
            _hidx_to_didx[ hidx     ] = new_didx;
            _didx_to_hidx[ new_didx ] = hidx;
        }

        // Concatenate blending list and update bone index accordingly
        std::vector<Cluster>::const_iterator it = tree_cu->_blending_list.begin();
        for(int i = 0; it != tree_cu->_blending_list.end(); ++it, ++i)
        {
            Cluster c = *it;
            c.first_bone += off_bone;
            // Convert in device representation
            Cluster_cu new_c( c );
            hd_blending_list[off_blist + i] = new_c;
            hd_cluster_data [off_blist + i]._bulge_strength = c.datas._bulge_strength;
        }
        // We store nb_pairs in the first element of the list
        assert(tree_cu->_blending_list.size() > 0); // unless we have no elements
        hd_blending_list[off_blist].nb_pairs      = tree_cu->_blending_list.size()/2;

        hd_offset[t].list_data = off_blist;

        off_blist += tree_cu->_blending_list.size();
        off_bone  += tree_cu->_bone_aranged.size();
    }

    // Upload to GPU
    hd_offset.update_device_mem();
    hd_blending_list.update_device_mem();
    hd_cluster_data. update_device_mem();
    assert( off_blist == s_blend_list );
}

// -----------------------------------------------------------------------------

/// Convert CPU representation to GPU
void update_device()
{
    unbind();
    
    // List of concatened bones for all skeletons in 'h_envs'.  Note that a bone may
    // appear in h_generic_bones more than once, if it's used in multiple skeletons.
    std::vector<const Bone*> h_generic_bones;
    update_device_tree(h_generic_bones);

    fill_separated_bone_types( h_generic_bones );
    update_device_grid();
    bind();
}

// -----------------------------------------------------------------------------

void clean_env()
{
    unbind();
    for(unsigned i = 0; i < h_envs.size(); ++i){
        delete h_envs[i];
    }

    h_envs.clear();
    _didx_to_hidx.clear();
    _hidx_to_didx.clear();
    hd_offset.erase();
    hd_offset.update_device_mem();
    hd_grid_blending_list.erase();
    hd_grid_blending_list.update_device_mem();
    hd_grid_data.erase();
    hd_grid_data.update_device_mem();
    hd_grid.erase();
    hd_grid.update_device_mem();
    hd_grid_bbox.erase();
    hd_grid_bbox.update_device_mem();
    hd_blending_list.erase();
    hd_blending_list.update_device_mem();
    hd_bone_arrays->clear();
    hd_bone_arrays->update_device_mem();
    delete hd_bone_arrays;
    hd_bone_arrays = 0;
    allocated = false;
}

// -----------------------------------------------------------------------------

void alloc_hd_grid()
{
    assert( binded );
    unbind();

    int total_size = 0;
    for(unsigned i = 0; i < h_envs.size(); ++i){
        if(h_envs[i] == NULL)
            continue;
        const int res = h_envs[i]->h_grid->res();
        total_size += res*res*res;
    }
    hd_grid.malloc(total_size, -1);
    hd_grid_bbox.malloc( h_envs.size() * 2 ); // Two points for a bbox

    bind();
}

// -----------------------------------------------------------------------------

void init_env()
{
    if( !allocated)
    {
        hd_bone_arrays = new Bone_tex();
        allocated = true;
        bind();
    }
}

// -----------------------------------------------------------------------------

Skel_id new_skel_instance(const std::vector<const Bone*>& bones,
                          const std::map<Bone::Id, Bone::Id>& parents,
                          int grid_res)
{
    SkeletonEnv *env = new SkeletonEnv();
    env->h_tree = new Tree(bones, parents);
    env->h_grid = new Grid(env->h_tree, grid_res);
    env->h_grid->build_grid();

    // Find an empty slot.
    int id;
    for(id = 0; id < (int) h_envs.size(); ++id)
    {
        if(h_envs[id] == NULL)
            break;
    }

    // Add a slot if needed.
    if(id >= h_envs.size())
        h_envs.push_back(NULL);
    
    h_envs[id] = env;

    alloc_hd_grid();
    update_device();
    return id;
}

// -----------------------------------------------------------------------------

void delete_skel_instance(Skel_id skel_id)
{
    assert(skel_id < h_envs.size());
    assert(skel_id >= 0);

    // Set the slot to NULL to allow reuse.
    delete h_envs[skel_id];
    h_envs[skel_id] = NULL;
    
    alloc_hd_grid();
    update_device();
}

// -----------------------------------------------------------------------------

void update_bones_data(Skel_id i)
{
    h_envs[i]->h_grid->build_grid();
    update_device();
}

// -----------------------------------------------------------------------------

void update_joints_data(Skel_id i, const std::map<Bone::Id, Joint_data>& joints)
{
    h_envs[i]->h_tree->set_joints_data( joints );
    h_envs[i]->h_grid->build_grid();
    update_device();
}

// -----------------------------------------------------------------------------

void set_grid_res(Skel_id i, int res)
{
    assert( res > 0);
    h_envs[i]->h_grid->set_res( res );
    alloc_hd_grid();
    update_device();
}

// -----------------------------------------------------------------------------

DBone_id bone_hidx_to_didx(Skel_id skel_id, Bone::Id bone_hidx)
{
    // TODO: array of maps by skeleton ids would be more efficient
    Hbone_id hidx(skel_id, bone_hidx);
    return Std_utils::find( _hidx_to_didx, hidx);
}

// -----------------------------------------------------------------------------

Bone::Id bone_didx_to_hidx(Skel_id skel_id, DBone_id bone_didx)
{
    Hbone_id hid = Std_utils::find( _didx_to_hidx, bone_didx);
    assert( hid._skel_id == skel_id);
    return hid._bone_id;
}
}// End Skeleton_env ===========================================================

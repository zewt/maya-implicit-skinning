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

// =============================================================================
namespace Skeleton_env {
// =============================================================================

bool allocated = false;

// from file skeleton_env_tex_binded.hpp
extern bool binded;

// -----------------------------------------------------------------------------
/// @name GPU friendly datas
// -----------------------------------------------------------------------------

Bone_tex* hd_bone_arrays = 0;

/// Concatenated blendind list for every skeletons
Cuda_utils::HD_Array<Cluster_cu> hd_blending_list;

Cuda_utils::HD_Array<Cluster_data> hd_cluster_data;

/// Concatenated blendind list for every skeletons in each grid cell not empty.
/// Each cell store a single blending list.
/**
 * @code
 *  |-list0-|-list1-|-list2-| |-list0-|-list1-|
 *  *-----------------------* *---------------*
 *            skel0                 skel1
 * @endcode
*/
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

/// List of concatened bones for every skeletons in 'h_tree_instances'
std::vector<Bone*> h_generic_bones;
/// user idx to device bone idx
std::map<Hbone_id, DBone_id> _hidx_to_didx;
/// device bone idx to user idx
std::map<DBone_id, Hbone_id> _didx_to_hidx;


/// List of skeletons instances, a skeleton must be a tree with one connex
/// componant.
std::deque<Tree*>    h_tree_instances;
/// Prepared tree data to go on GPU
std::deque<Tree_cu*> h_tree_cu_instances;

/// Acceleration structure for the different skeletons in the environment
std::deque<Grid*> h_grid;



// =============================================================================

/// @param sid : skeleton identifier
/// @param cell_id : linear index of the 3d cell we want to extract the blending
/// list
/// @param blist : described the sub-skeleton in the
void cell_to_blending_list(Skel_id sid,
                           int cell_id,
                           std::list<Cluster>& blist)
{
    // Note: if in each cell the list of bones is order from root to leaf
    // then blending list will be also ordered root to leaf
    const Grid*    g    = h_grid[sid];
    const Tree_cu* tree = h_tree_cu_instances[sid];
    const std::list<Bone::Id>& bones_in_cell = g->_grid_cells[cell_id];

    std::vector<bool> cluster_done;
    cluster_done.resize(tree->_clusters.size(), false);

    Tree_cu::BList blending_list( *(h_tree_cu_instances[sid]) );
    std::list<Bone::Id>::const_iterator bones_it = bones_in_cell.begin();
    for(; bones_it != bones_in_cell.end(); ++bones_it)
    {
        DBone_id dbone = bone_hidx_to_didx(sid, *bones_it);

        Cluster_id clus_id = tree->bone_to_cluster( dbone );
        if( cluster_done[clus_id.id()] ) continue;

        blending_list.add_cluster( clus_id );

        cluster_done[clus_id.id()] = true;
    }

    // HACK: this should be done outside this function when convberting cluster
    // to cluster_cu
    if(blending_list._list.size() > 0){
        blending_list._list.begin()->datas._ctrl_id    = blending_list._nb_singletons;
        blending_list._list.begin()->datas._blend_type = (EJoint::Joint_t)blending_list._nb_pairs;
    }

    blist.swap( blending_list._list );
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
    std::deque< std::list<Cluster>* > cells_blist; // TODO/ <- pre-allocate this in alloc_hd_grid()
    int offset = 0;
    int grid_offset = 0;
    for(unsigned grid_id = 0; grid_id < h_grid.size(); ++grid_id)
    {
        Grid* grid = h_grid[grid_id];
//        grid->build_grid(); should be already done

        std::set<int>::const_iterator it = grid->_filled_cells.begin();
        for( ; it != grid->_filled_cells.end(); ++it)
        {
            int cell_idx = *it;

            std::list<Cluster>* blist = new std::list<Cluster>();
            cells_blist.push_back( blist );
            cell_to_blending_list(grid_id, cell_idx, *blist );

            hd_grid[grid_offset + cell_idx] = offset;
            offset += (int)blist->size();
        }

        hd_offset[grid_id].grid_data = grid_offset;
        const int res = grid->res();
        grid_offset += res*res*res;

        // Update grid bbox and resolution
        BBox_cu bb = grid->bbox();
        hd_grid_bbox[grid_id*2 + 0] = bb.pmin.to_float4();
        hd_grid_bbox[grid_id*2 + 1] = bb.pmax.to_float4();
        hd_grid_bbox[grid_id*2 + 0].w = (float)res;
    }

    hd_grid_blending_list.malloc( offset );
    hd_grid_data.malloc( offset );

    // Concatenate blending lists compute new concatenated bone index:
    int grid_id = 0;
    int off_bone = 0;
    int blist_offset = 0;
    for(unsigned i = 0; i < cells_blist.size(); ++i)
    {
        const std::list<Cluster>& blist = *(cells_blist[i]);
        std::list<Cluster>::const_iterator it = blist.begin();
        for(unsigned l = 0; it != blist.end(); ++it, ++l)
        {

            // Convert cluster to cluster_cu and offset bones id to match the concateneted representation
            Cluster c = *it;
            Cluster_cu clus(c);

            clus.first_bone += off_bone;

            hd_grid_blending_list[blist_offset + l] = clus;
            hd_grid_data         [blist_offset + l]._bulge_strength = c.datas._bulge_strength;;
        }

        blist_offset += blist.size();
        delete cells_blist[i];
        cells_blist[i] = 0;

        // Compute grid identifier and bone offset
        int res = h_grid[grid_id]->res();
        if( (hd_offset[grid_id].grid_data +  res*res*res) < ((int)i+1)){
            grid_id++;
            if( grid_id < (int)h_grid.size() )
                off_bone += h_tree_cu_instances[grid_id]->_bone_aranged.size();
        }
    }

    hd_offset.update_device_mem(); // This is also done in update_device_tree maybe we can factorize
    hd_grid.update_device_mem();
    hd_grid_blending_list.update_device_mem();
    hd_grid_data.update_device_mem();
    hd_grid_bbox.update_device_mem();
#endif
}

// -----------------------------------------------------------------------------

/// Fill device array : hd_bone_types; hd_bone; hd_bone_hrbf; hd_bone_cylinder;
/// hd_bone_precomputed; hd_bulge_strength;
static void fill_separated_bone_types(const std::vector<Bone*>& generic_bones)
{
    assert( !binded   );
    assert( allocated );

    const int nb_bones = generic_bones.size();
    hd_bone_arrays->resize( nb_bones );
    for(int i = 0; i < nb_bones; i++)
    {
        const Bone* b = generic_bones[i];
        const int btype = b->get_type();
        switch( btype )
        {
        case EBone::CYLINDER:
            hd_bone_arrays->hd_bone_cylinder[i] = ((Bone_cylinder*)b)->get_cylinder();
            break;
        case EBone::HRBF:
            hd_bone_arrays->hd_bone_hrbf[i] = ((Bone_hrbf*)b)->get_hrbf();
            break;
        case EBone::SSD:
            /* There is no data to store for ssd bones*/
            break;
        case EBone::PRECOMPUTED:
            hd_bone_arrays->hd_bone_precomputed[i] = ((Bone_precomputed*)b)->get_primitive();
            break;
        }

        hd_bone_arrays->hd_bone      [i] = b->get_bone_cu();
        hd_bone_arrays->hd_bone_types[i] = btype;
    }
    // Upload every arrays to GPU
    hd_bone_arrays->update_device_mem();
}

// -----------------------------------------------------------------------------


/// Fill device array : hd_blending_list; hd_offset (only list_data field);
/// h_generic_bones; _hidx_to_didx; _didx_to_hidx;
static void update_device_tree()
{
    assert( !binded );
    // Convert host layout to the GPU friendly layout
    // And compute some array sizes.
    std::deque<Tree_cu*>& tree_cu_list = h_tree_cu_instances;

    int nb_bones_all = 0; // Number of bones for every concatenated skels
    int s_blend_list = 0; // Size of the concatenated blending list
    for(unsigned i = 0; i < h_tree_instances.size(); ++i)
    {
        // Convert tree to GPU layout
        delete tree_cu_list[i];
        tree_cu_list[i] = new Tree_cu( h_tree_instances[i] );
        nb_bones_all += h_tree_instances[i]->bone_size();
        s_blend_list += tree_cu_list[i]->_blending_list._list.size();
    }

    // Now we can allocate memory
    hd_offset.malloc( h_tree_instances.size() );
    h_generic_bones.resize( nb_bones_all );

    hd_blending_list.malloc( s_blend_list );
    hd_cluster_data. malloc( s_blend_list );

    _hidx_to_didx.clear();
    _didx_to_hidx.clear();

    // Concatenate bones and blending list.
    // Note that the bone identifiers in the new blending list must
    // be changed to match the list of concatenated bones

    int off_bone  = 0; // Offset to store bones in h_bone_device
    int off_blist = 0; // Offset to store blending list in
    for(unsigned t = 0; t < h_tree_instances.size(); ++t)
    {
        const Tree_cu* tree_cu = tree_cu_list[t];

        for(unsigned i = 0; i < tree_cu->_bone_aranged.size(); ++i){
            DBone_id new_didx = DBone_id(i) + off_bone;
            Hbone_id hidx(t, tree_cu->get_id_bone_aranged( i ) );
            h_generic_bones[new_didx.id()] = tree_cu->_bone_aranged[i];
            // Build correspondance between device/host index for the
            // concatenated bones
            _hidx_to_didx[ hidx     ] = new_didx;
            _didx_to_hidx[ new_didx ] = hidx;
        }

        // Concatenate blending list and update bone index accordingly
        std::list<Cluster>::const_iterator it = tree_cu->_blending_list._list.begin();
        for(int i = 0; it != tree_cu->_blending_list._list.end(); ++it, ++i)
        {
            Cluster c = *it;
            c.first_bone += off_bone;
            // Convert in device representation
            Cluster_cu new_c( c );
            hd_blending_list[off_blist + i] = new_c;
            hd_cluster_data [off_blist + i]._bulge_strength = c.datas._bulge_strength;
        }
        // We store nb_pairs and nb_singletons in the first element of the list
        hd_blending_list[off_blist].nb_pairs      = tree_cu->_blending_list._nb_pairs;
        hd_blending_list[off_blist].nb_singletons = tree_cu->_blending_list._nb_singletons;

        hd_offset[t].list_data = off_blist;

        off_blist += tree_cu->_blending_list._list.size();
        off_bone  += tree_cu->_bone_aranged.size();
    }

    // Upload to GPU
    hd_offset.update_device_mem();
    hd_blending_list.update_device_mem();
    hd_cluster_data. update_device_mem();
    assert( off_bone  == nb_bones_all );
    assert( off_blist == s_blend_list );
}

// -----------------------------------------------------------------------------

/// Convert CPU representation to GPU
void update_device()
{
    unbind();
    update_device_tree();
    fill_separated_bone_types( h_generic_bones );
    update_device_grid();
    bind();
}

// -----------------------------------------------------------------------------

void clean_env()
{
    unbind();
    for(unsigned i = 0; i < h_tree_instances.size(); ++i){
        delete h_tree_instances[i];
        delete h_tree_cu_instances[i];
        delete h_grid[i];
    }

    h_tree_instances.clear();
    h_tree_cu_instances.clear();
    h_grid.clear();
    _didx_to_hidx.clear();
    _hidx_to_didx.clear();
    h_generic_bones.clear();
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
    for(unsigned i = 0; i < h_grid.size(); ++i){
        const int res = h_grid[i]->res();
        total_size += res*res*res;
    }
    hd_grid.malloc(total_size, -1);
    hd_grid_bbox.malloc( h_grid.size() * 2 ); // Two points for a bbox

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

Skel_id new_skel_instance(Bone::Id root_idx,
                          const std::vector<Bone*>& bones,
                          const std::vector<int>& parents)
{
    int id = (unsigned)h_tree_instances.size();

    Tree* tree = new Tree(root_idx, bones, parents);
    h_tree_instances.push_back( tree );
    h_tree_cu_instances.push_back( 0 );
    h_grid.push_back( new Grid( tree ) );
    alloc_hd_grid();
    update_device();
    return id;
}

// -----------------------------------------------------------------------------

void delete_skel_instance(Skel_id i)
{
    delete h_grid[i];
    delete h_tree_instances[i];
    delete h_tree_cu_instances[i];
    h_tree_instances.erase( h_tree_instances.begin() +i);
    h_tree_cu_instances.erase( h_tree_cu_instances.begin() + i);
    h_grid.erase( h_grid.begin() + i);
    alloc_hd_grid();
    update_device();
}

// -----------------------------------------------------------------------------

void update_bones_data(Skel_id i, const std::vector<Bone*>& bones)
{
    h_tree_instances[i]->set_bones( bones );
    h_grid[i]->build_grid();
    update_device();
}

// -----------------------------------------------------------------------------

void update_joints_data(Skel_id i, const std::vector<Joint_data>& joints)
{
    h_tree_instances[i]->set_joints_data( joints );
    h_grid[i]->build_grid();
    update_device();
}

// -----------------------------------------------------------------------------

void set_grid_res(Skel_id i, int res)
{
    assert( res > 0);
    h_grid[i]->set_res( res );
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

// -----------------------------------------------------------------------------
/// Debugging
// -----------------------------------------------------------------------------

#include "blending_env_tex.hpp"
#include "skeleton_env_tex.hpp"

#include "vbo_primitives.hpp"
#include "glsave.hpp"
#include"color.hpp"

// Some externs from "globals.hpp" ---------------------------------------------
extern VBO_primitives g_primitive_printer;
extern Prim_id g_cube_vbo;
// End externs from "globals.hpp" ----------------------------------------------

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// Draw a grid cell (red if empty or green)
void draw_cell(const Point_cu& org, Vec3_cu lengths, bool empty)
{
    glPushMatrix();

    glTranslatef(org.x, org.y, org.z);
    glScalef(lengths.x, lengths.y, lengths.z);

    if( empty ) glColor3f(1.f, 0.f, 0.f);
    else        glColor3f(0.f, 1.f, 0.f);
    g_primitive_printer.draw( g_cube_vbo );
    glPopMatrix();
}



// -----------------------------------------------------------------------------

/// Evaluate a point given a skeleton's grid
void draw_sample(Skel_id i, Vec3_cu pos)
{
    Cluster_id clus_off = fetch_grid_blending_list_offset(i, pos);

    if( clus_off.is_valid() ){
        Cluster_cu clus = fetch_grid_blending_list( clus_off );
        // In the first cluster we don't store the blending type and controller id
        const int nb_pairs      = clus.nb_pairs;
        const int nb_singletons = clus.nb_singletons;

//        if(nb_pairs == 0 && nb_singletons == 0) glColor3f(0.f, 0.f, 0.f);
//        else                                    glColor3f(0.f, 1.f, 0.f);

        float c = (float)(nb_pairs*2 + nb_singletons) / (float)(h_tree_instances[i]->bone_size());
        Color::heat_color(1.f - c).set_gl_state();

        /*
        int cid = 0;
        for(; cid < nb_pairs; ++cid) {
            Cluster_cu clus0 = fetch_grid_blending_list( clus_off + cid*2 + 0);
            Cluster_cu clus1 = fetch_grid_blending_list( clus_off + cid*2 + 0);
            int a = 0;
            a++;
        }

        cid *= 2;
        int off = cid;
        for(; cid < (nb_singletons + off); ++cid){
            Cluster_cu clus0 = fetch_grid_blending_list( clus_off + cid);
            int a = 0;
            a++;
        }
        */
    }
    else
        glColor3f(1.f, 0.f, 0.f);

    glBegin(GL_POINTS);
    glVertex3f(pos.x, pos.y, pos.z);
    glEnd();
}

// -----------------------------------------------------------------------------

void draw_grids()
{
    GLPolygonModeSave poly_mode( GL_LINE );
    GLEnabledSave light(GL_LIGHTING, true, false);
    GLEnabledSave depth(GL_DEPTH_TEST, true, true);
    GLPointSizeSave size(5.f);
    for(unsigned gid = 0; gid < h_grid.size(); ++gid)
    {
        BBox_cu bb;
        int res;

        float4 f0 = hd_grid_bbox[gid*2 + 0];
        float4 f1 = hd_grid_bbox[gid*2 + 1];

        bb = BBox_cu( Point_cu(f0.x, f0.y, f0.z),
                      Point_cu(f1.x, f1.y, f1.z) );
        res = (int)f0.w;

        Vec3i_cu size_grid(res, res, res);
        Vec3_cu cell_lengths = bb.lengths().div( (Vec3_cu)size_grid );
        for(Idx3_cu idx(size_grid, 0); idx.is_in(); ++idx)
        {
            Point_cu pt = bb.pmin + cell_lengths.mult( (Vec3_cu)idx.to_3d() );
//            int offset = hd_offset[gid].grid_data;
//            int state = hd_grid[ offset + idx.to_linear() ];
//            draw_cell(pt, cell_lengths, state < 0);

//            const int cpt = 10;
//            for (int i = 0; i < cpt; ++i)
                draw_sample(gid, pt + (cell_lengths / (float)2.f) /** i*/);

        }

    }
}

}// End Skeleton_env ===========================================================

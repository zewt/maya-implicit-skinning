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
#include "precomputed_prim_env.hpp"

// -----------------------------------------------------------------------------

#include "bone.hpp"
#include "cuda_utils.hpp"
#include "transfo.hpp"
#include "bbox.hpp"
#include "animesh_kers.hpp"

// -----------------------------------------------------------------------------

#include "precomputed_prim_constants.hpp"
#include "precomputed_prim_kernels.hpp"

// -----------------------------------------------------------------------------

#include <deque>
#include <iostream>

// -----------------------------------------------------------------------------

// Forward def skeleton_env.hpp
#include "skeleton_env_type.hpp"
namespace Skeleton_env{
    extern DBone_id bone_hidx_to_didx(Skel_id skel_id, Bone::Id bone_hidx);
}

// =============================================================================
namespace Precomputed_env{
// =============================================================================

using namespace Cuda_utils;

/// This array is updated by update_device_transformations()
/// d_anim_transform[i] == h_grid_transfo_buffer[i]
Device::Array<Transfo> d_anim_transform;

/// d_grad_transform[i] == h_user_transform[i]
Device::Array<Transfo> d_grad_transform;

/// Temporary buffer used to transfer 'h_grid_transform' rapidly to 'd_anim_transform'
/// The buffer is filled with set_transform()
/// h_grid_transfo_buffer[i] = h_grid_transform[i] * h_user_transform[i]
Host::PL_Array<Transfo> h_grid_transfo_buffer;

/// Transformation associated to a grid in initial position.
/// point_in_grid_space = h_grid_transform[i] * point_in_world_space;
Host::Array<Transfo> h_grid_transform;

/// Transformation set by the user for every grid.
Host::Array<Transfo> h_user_transform;

Device::CuArray<float4> d_block;

/// Array of grids : d_grids[inst_id][grid_element]
std::deque< DA_float4* > d_grids;

DA_int4 d_offset;
HA_int4 h_offset;

int nb_instances = 0;

texture<float4, 3, cudaReadModeElementType> tex_grids;
texture<float4, 1, cudaReadModeElementType> tex_transform;
texture<float4, 1, cudaReadModeElementType> tex_transform_grad;
texture<int4  , 1, cudaReadModeElementType> tex_offset_;

// -----------------------------------------------------------------------------

extern void unbind();
extern void bind();

// -----------------------------------------------------------------------------

void clean_env()
{
    unbind();
    d_anim_transform.erase();
    d_grad_transform.erase();
    h_grid_transfo_buffer.erase();
    h_grid_transform.erase();
    h_user_transform.erase();
    d_block.erase();
    d_grids.clear();
    d_offset.erase();
    h_offset.erase();
    nb_instances = 0;
}

// -----------------------------------------------------------------------------

/// Fills the 'out_block' array with the grid 'in_grid'
__global__
void fill_block(DA_float4 in_grid, int3 org, int3 block_size, DA_float4 out_block)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < in_grid.size())
    {
        // Local (x, y, z) coordinates in the grid
        int3 grid = { idx % GRID_RES,
                     (idx / GRID_RES) % GRID_RES,
                      idx / (GRID_RES * GRID_RES) };

        // Coordinates in the block
        int3 block = {org.x + grid.x, org.y + grid.y, org.z + grid.z};
        // Convert 'block' to the linear indice in the array
        int idx_block =
                block.x +
                block.y * block_size.x +
                block.z * block_size.x * block_size.y;

        out_block[idx_block] = in_grid[idx];
    }
}

// -----------------------------------------------------------------------------

void copy_grids_to_cuarray_block()
{
    assert(!binded);

    if(h_offset.size() == 0)
        return;

    int3 block_size = {0, 0, 0};
    for(int i = 0; i < h_offset.size(); i++)
    {
        if(h_offset[i].x < 0)
            continue;

        block_size.x = max(h_offset[i].x*GRID_RES + GRID_RES, block_size.x);
        block_size.y = max(h_offset[i].y*GRID_RES + GRID_RES, block_size.y);
        block_size.z = max(h_offset[i].z*GRID_RES + GRID_RES, block_size.z);
    }

    DA_float4 block_temp(block_size.x*block_size.y*block_size.z);

    // Looping through all grid's instances
    for(int i = 0; i < h_offset.size(); i++)
    {
        // (x, y, z) indices in the block :
        int4 off = h_offset[i];
        if(off.x < 0)
            continue;

        printf("copy_grids_to_cuarray_block %i, %i, size %i\n", i, h_offset[i].x, d_grids[i]->size());
        fflush(stdout);
        // Grid's origin (x, y, z) coordinates in the block
        int3 org = {off.x * GRID_RES, off.y * GRID_RES, off.z * GRID_RES };

        int ker_b_size = 64;
        int ker_g_size = ((*d_grids[i]).size() + ker_b_size - 1) / ker_b_size;

        // Filling all grid's element of instance 'i'
        fill_block<<<ker_g_size, ker_b_size >>>(*d_grids[i], org, block_size, block_temp);
    }

    d_block.malloc(block_size.x, block_size.y, block_size.z);
    d_block.copy_from(block_temp.ptr(), block_temp.size());
}

// -----------------------------------------------------------------------------

static void update_offset(int idx)
{
    const int block_length = MAX_TEX_LENGTH / GRID_RES;

    int4 grid_index = { idx % block_length,
                       (idx / block_length) % block_length,
                        idx / (block_length * block_length),
                        0 };

    h_offset[idx] = grid_index;
    d_offset.copy_from(h_offset);
}

// -----------------------------------------------------------------------------

int new_instance()
{
    assert(nb_instances >= 0);

    Precomputed_env::unbind();

    // find the first free element (which is represented by negative offsets)
    int idx = 0;
    for(; idx<h_offset.size(); idx++)
        if( h_offset[idx].x < 0) break;

    // Add the instance
    if(idx == h_offset.size())
    {
        const int size = h_offset.size() + 1;
        h_offset.realloc( size );
        d_grids.push_back(new DA_float4(GRID_RES_3));
        d_offset.realloc( size );
        h_grid_transform.realloc(size);
        h_user_transform.realloc(size);
        h_grid_transfo_buffer.realloc(size);
        d_anim_transform.realloc(size);
        d_grad_transform.realloc(size);
    }
    else
        d_grids[idx] = new DA_float4(GRID_RES_3);

    update_offset(idx);

    nb_instances++;

    Precomputed_env::bind();
    return idx;
}

// -----------------------------------------------------------------------------

void delete_instance(int inst_id)
{
    assert(inst_id < h_offset.size());
    assert(inst_id >= 0);
    assert(h_offset[inst_id].x >= 0);
    assert(nb_instances > 0);
    Precomputed_env::unbind();

    delete d_grids[inst_id];

    if(inst_id == (h_offset.size()-1))
    {
        const int size = h_offset.size() - 1;
        h_offset.erase(size);
        d_offset.erase(size);
        d_grids.pop_back();
        h_grid_transform.erase(size);
        h_user_transform.erase(size);
        h_grid_transfo_buffer.erase(size);
        d_anim_transform.erase(size);
        d_grad_transform.erase(size);
    }
    else
    {
        // Deleted hrbf instances are tag with negative offsets in order to
        // re-use the element for a new instance
        h_offset[inst_id] = make_int4(-1, -1, -1, -1);
    }
    d_offset.copy_from(h_offset);

    // OK maybe its a bit slow to do it each time.
    // The best would be to do it once when all grids are loadeds
    copy_grids_to_cuarray_block();

    nb_instances--;

    Precomputed_env::bind();
}

// -----------------------------------------------------------------------------

void reset_instance(int inst_id)
{
    delete_instance(inst_id);
    Precomputed_env::unbind();
    if(inst_id == h_offset.size())
    {
        const int size = h_offset.size() + 1;
        h_offset.realloc( size );
        d_grids.push_back(new DA_float4(GRID_RES_3));
        d_offset.realloc( size );
        h_grid_transform.realloc(size);
        h_user_transform.realloc(size);
        h_grid_transfo_buffer.realloc(size);
        d_anim_transform.realloc(size);
        d_grad_transform.realloc(size);
    }
    else
        d_grids[inst_id] = new DA_float4(GRID_RES_3);

    update_offset( inst_id );
    nb_instances++;
    Precomputed_env::bind();
}

// -----------------------------------------------------------------------------

/// Give the transformation from world coordinates to the grid defined
/// by the bouding box 'bb' of resolution 'res'
static Transfo world_coord_to_grid(const OBBox_cu& obbox, int res)
{
    Vec3_cu v = obbox._bb.pmax - obbox._bb.pmin;
    float3 steps = {(float)res / v.x, (float)res / v.y, (float)res / v.z};

    Mat3_cu scale = Mat3_cu(steps.x, 0.f    , 0.f,
                            0.f    , steps.y, 0.f,
                            0.f    , 0.f    , steps.z);

    return Transfo::translate( Vec3_cu(0.5f, 0.5f, 0.5f) ) * // Because Cuda texture is going to be filtered (trilinear) we need to offset
           Transfo(scale) *                                  // Scale so the point coordinates matches the grid units lengths
           Transfo::translate( -(Vec3_cu)obbox._bb.pmin ) *  // translate to the grid origin
           obbox._tr.fast_invert();                          // To box coordinates
}


// -----------------------------------------------------------------------------

static void fill_grid(Bone::Id bone_id,
                      Skeleton_env::Skel_id skel_id,
                      const OBBox_cu& obbox,
                      int res,
                      DA_float4& d_grid)
{
    assert(GRID_RES_3 == d_grid.size());

    Vec3_cu lengths = obbox._bb.lengths();
    float3  steps = {lengths.x / (float)res,
                     lengths.y / (float)res,
                     lengths.z / (float)res};

    const int ker_block_size = 64;
    const int ker_grid_size  =
            (d_grid.size() + ker_block_size - 1) / ker_block_size;


    if(ker_grid_size > 65535){
        std::cerr << "ERROR: The grid is too large cuda thread size is exceeded." << std::endl;
        assert(false);
    }

    using namespace Skeleton_env;

    DBone_id device_bone_id = bone_hidx_to_didx(skel_id, bone_id);

    fill_grid_with_fngf(device_bone_id,
                        steps,
                        res,
                        obbox._bb.pmin,
                        obbox._tr,
                        ker_grid_size,
                        ker_block_size,
                        d_grid);

    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void init_instance(int inst_id, Skeleton_env::Skel_id _skel_id, const Bone* bone)
{
    assert(inst_id < h_offset.size());
    assert(inst_id >= 0);
    assert(h_offset[inst_id].x >= 0); // means the instance was deleted
    assert(nb_instances > 0);

    reset_instance(inst_id);

    Precomputed_env::unbind();

    Bone::Id bone_id = bone->get_bone_id();
    OBBox_cu obbox = bone->get_obbox();

    // Compute the primive's grid
    fill_grid(bone_id, _skel_id, obbox, GRID_RES, (*d_grids[inst_id]));

    // Adding the transformation to evaluate the grid
    Transfo t = world_coord_to_grid(obbox, GRID_RES);
    d_anim_transform.set(inst_id, t);
    d_grad_transform.set(inst_id, Transfo::identity());
    h_grid_transform[inst_id] = t;
    h_user_transform[inst_id] = Transfo::identity();

    // OK maybe its a bit slow to do it each time.
    // The best would be to do it once when all grids are loadeds
    copy_grids_to_cuarray_block();

    Precomputed_env::bind();
}

// -----------------------------------------------------------------------------

const Transfo& get_user_transform(int inst_id)
{
    return h_user_transform[inst_id];
}

// -----------------------------------------------------------------------------

void set_transform(int inst_id, const Transfo& transfo)
{
    h_grid_transfo_buffer[inst_id] = h_grid_transform[inst_id] * transfo.fast_invert();
    h_user_transform[inst_id] = transfo;
    d_grad_transform.set(inst_id, transfo);
}

// -----------------------------------------------------------------------------

void update_device_transformations()
{
    Precomputed_env::unbind();
    if(h_grid_transfo_buffer.size() > 0)
        d_anim_transform.copy_from(h_grid_transfo_buffer);
    Precomputed_env::bind();
}

}
// END PRECOMPUTED_ENV NAMESPACE ===============================================




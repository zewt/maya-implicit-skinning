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
#include <cassert>
#include <fstream>
#include <limits>
#include <iostream>

// -----------------------------------------------------------------------------

#include "cuda_utils.hpp"
#include "hrbf_env.hpp"
#include "hrbf_wrapper.hpp"
#include "hrbf_kernels.hpp"

namespace { __device__ void fix_debug() { } }

// -----------------------------------------------------------------------------

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

// =============================================================================
namespace HRBF_env{
// =============================================================================

using namespace Cuda_utils;

HDA_float4 hd_points;
HDA_float4 hd_alphas_betas;

DA_int2 d_offset;
HA_int2 h_offset;

DA_float4 d_init_points;
DA_float4 d_init_alpha_beta;
HA_Vec3_cu h_normals;

HDA_float hd_radius;

/// Transformations associated to each HRBF instances
HD_Array<Transfo> hd_transfo;

/// Maps elements of d_init_alpha_beta, d_init_points, etc.  to transformations
/// in hd_transfo
DA_int d_map_transfos;

int nb_hrbf_instance = 0;

/// Are textures currently binded with arrays
extern bool binded;

texture<float4, 1, cudaReadModeElementType> tex_points;
texture<float4, 1, cudaReadModeElementType> tex_alphas_betas;
texture<int2, 1, cudaReadModeElementType> tex_offset;
texture<float, 1, cudaReadModeElementType> tex_radius;

bool binded = false;


static void setup_tex()
{
    // tex_points setup
    tex_points.addressMode[0] = cudaAddressModeWrap;
    tex_points.addressMode[1] = cudaAddressModeWrap;
    tex_points.filterMode = cudaFilterModePoint;
    tex_points.normalized = false;
    // tex_alphas_betas setup
    tex_alphas_betas.addressMode[0] = cudaAddressModeWrap;
    tex_alphas_betas.addressMode[1] = cudaAddressModeWrap;
    tex_alphas_betas.filterMode = cudaFilterModePoint;
    tex_alphas_betas.normalized = false;
    // tex_offset setup
    tex_offset.addressMode[0] = cudaAddressModeWrap;
    tex_offset.addressMode[1] = cudaAddressModeWrap;
    tex_offset.filterMode = cudaFilterModePoint;
    tex_offset.normalized = false;
    // tex_radius setup
    tex_radius.addressMode[0] = cudaAddressModeWrap;
    tex_radius.addressMode[1] = cudaAddressModeWrap;
    tex_radius.filterMode = cudaFilterModePoint;
    tex_radius.normalized = false;
}

/// Bind hermite array data.
void bind()
{
    binded = true;
    
    setup_tex();
    d_offset.bind_tex( tex_offset );
    hd_alphas_betas.device_array().bind_tex( tex_alphas_betas );
    hd_points.      device_array().bind_tex( tex_points       );
    hd_radius.      device_array().bind_tex( tex_radius       );
}

// -----------------------------------------------------------------------------

void unbind()
{
    binded = false;
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_points)       );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_alphas_betas) );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_offset)       );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_radius)       );
}

void clean_env()
{
    HRBF_env::unbind();
    nb_hrbf_instance = 0;
    hd_points.erase();
    hd_points.update_device_mem();
    hd_alphas_betas.erase();
    hd_alphas_betas.update_device_mem();
    d_offset.erase();
    h_offset.erase();
    d_init_points.erase();
    d_init_alpha_beta.erase();
    h_normals.erase();
    hd_radius.erase();
    hd_radius.update_device_mem();
    hd_transfo.erase();
    hd_transfo.update_device_mem();
    d_map_transfos.erase();
}

// -----------------------------------------------------------------------------

void reset_env()
{
    clean_env();
}

// -----------------------------------------------------------------------------

void get_instance_id_list(std::vector<int>& list_id){
    list_id.clear();

    for(int i = 0; i < h_offset.size(); i++)
        if(h_offset[i].x >= 0) // Negative instance means it has be deleted
            list_id.push_back(i);
}

// -----------------------------------------------------------------------------

int get_instance_size(int id){
    assert( id < h_offset.size());
    assert( id >= 0 );
    assert( h_offset[id].x >= 0 );

    return h_offset[id].y;
}

// -----------------------------------------------------------------------------

int get_offset(int id){
    assert( id >= 0 );
    assert( id < h_offset.size());
    assert( h_offset[id].x >= 0 );

    return h_offset[id].x;
}

// -----------------------------------------------------------------------------

Point_cu get_sample(int hrbf_id, int sample_idx){
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( sample_idx < get_instance_size(hrbf_id) );
    assert( sample_idx >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    HRBF_env::unbind();
    float4 tmp = d_init_points.fetch( h_offset[hrbf_id].x + sample_idx );
    HRBF_env::bind();
    return Point_cu(tmp.x, tmp.y, tmp.z);
}

// -----------------------------------------------------------------------------

void get_anim_samples(int hrbf_id, std::vector<Point_cu>& samp_list)
{
    assert(binded);
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    int inst_size = h_offset[hrbf_id].y;
    samp_list.resize(inst_size);
    if(inst_size > 0)
    {
        /* //TODO: to be deleted
        HA_float4 h_temp(inst_size);
        HRBF_env::unbind();
        mem_cpy_dth(h_temp.ptr(), d_points.ptr()+h_offset[hrbf_id].x, inst_size);
        HRBF_env::bind();

        for(int i = 0; i < inst_size; i++)
            samp_list[i] = Point_cu(h_temp[i].x, h_temp[i].y, h_temp[i].z);
      */
        for(int i = 0; i < inst_size; i++)
        {
            float4 p = hd_points[ i + h_offset[hrbf_id].x];
            samp_list[i] = Point_cu(p.x, p.y, p.z);
        }
    }
}

// -----------------------------------------------------------------------------

void get_anim_weights(int hrbf_id, std::vector<float4>& weights_list)
{
    assert(binded);
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    int inst_size = h_offset[hrbf_id].y;
    weights_list.resize(inst_size);
    if(inst_size > 0)
    {
        mem_cpy_hth(&(weights_list[0]),
                    hd_alphas_betas.ptr() + h_offset[hrbf_id].x,
                    inst_size);
    }
}

// -----------------------------------------------------------------------------

void get_samples(int hrbf_id, std::vector<Vec3_cu>& samp_list)
{
    assert(binded);
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    int inst_size = h_offset[hrbf_id].y;
    samp_list.resize(inst_size);
    if(inst_size > 0)
    {
        HA_float4 h_temp(inst_size);
        HRBF_env::unbind();
        mem_cpy_dth(h_temp.ptr(), d_init_points.ptr()+h_offset[hrbf_id].x, inst_size);
        HRBF_env::bind();

        for(int i = 0; i < inst_size; i++)
            samp_list[i] = Vec3_cu(h_temp[i].x, h_temp[i].y, h_temp[i].z);
    }
}

// -----------------------------------------------------------------------------

void get_weights(int hrbf_id, std::vector<float4>& weights_list)
{
    assert(binded);
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    int inst_size = h_offset[hrbf_id].y;
    weights_list.resize(inst_size);
    if(inst_size > 0)
    {
        HRBF_env::unbind();
        mem_cpy_dth(&(weights_list[0]), d_init_alpha_beta.ptr()+h_offset[hrbf_id].x, inst_size);
        HRBF_env::bind();
    }

}

// -----------------------------------------------------------------------------

Vec3_cu get_normal(int hrbf_id, int sample_idx)
{
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( sample_idx < get_instance_size(hrbf_id) );
    assert( sample_idx >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    return h_normals[ h_offset[hrbf_id].x + sample_idx ];
}

// -----------------------------------------------------------------------------

void get_normals(int hrbf_id, std::vector<Vec3_cu>& normal_list)
{
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    int inst_size = h_offset[hrbf_id].y;
    normal_list.resize(inst_size);
    if(inst_size > 0)
    {
        for(int i = 0; i < inst_size; i++)
            normal_list[i] = h_normals[i+h_offset[hrbf_id].x];
    }
}

// -----------------------------------------------------------------------------

void get_anim_normals(int hrbf_id, std::vector<Vec3_cu>& normal_list)
{
    get_normals(hrbf_id, normal_list);

    Transfo tr = get_transfo( hrbf_id );
    Transfo tr_nor = tr.fast_invert().transpose();

    for(unsigned i = 0; i < normal_list.size(); ++i)
        normal_list[i] = tr_nor * normal_list[i];
}

// -----------------------------------------------------------------------------

Vec3_cu get_anim_normal(int hrbf_id, int sample_idx)
{
    Transfo tr = get_transfo( hrbf_id );
    Transfo tr_nor = tr.fast_invert().transpose();

    return tr_nor * get_normal(hrbf_id, sample_idx);
}

// -----------------------------------------------------------------------------

float4 get_weights(int hrbf_id, int sample_idx){
    assert( hrbf_id < h_offset.size() );
    assert( hrbf_id >= 0 );
    assert( sample_idx < get_instance_size(hrbf_id) );
    assert( sample_idx >= 0 );
    assert( h_offset[hrbf_id].x >= 0 );

    HRBF_env::unbind();
    float4 f = d_init_alpha_beta.fetch( h_offset[hrbf_id].x + sample_idx);
    HRBF_env::bind();
    return f;
}

// -----------------------------------------------------------------------------

float get_inst_radius(int hrbf_id)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );

    HRBF_env::unbind();
    float radius = 0.f;
    radius = hd_radius[hrbf_id];
    HRBF_env::bind();
    return radius;
}

// -----------------------------------------------------------------------------

Transfo get_transfo(int hrbf_id)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert( hd_transfo.size() == h_offset.size());

    // We don't need to bind/unbind transdformations to texture
    //HRBF_env::unbind();
    Transfo tr = hd_transfo[hrbf_id];
    //HRBF_env::bind();
    return tr;
}

// -----------------------------------------------------------------------------

/// Private function
/// re-compute every elements of h_offset[].x given the value in
/// h_offset[].y and setting h_offset[hrbf_id].y = new_size;
static void update_offset(int hrbf_id, int new_size)
{
    assert(!HRBF_env::binded);
    assert(d_offset.size() == h_offset.size());
    assert(!binded);

    h_offset[hrbf_id].x = 0;
    h_offset[hrbf_id].y = new_size;

    int acc = 0;
    for(int i = 0; i < h_offset.size(); i++ )
    {
        // Negative offset means the instance is deleted and we need to keep
        // negative to use the spot again later
        if(h_offset[i].x >= 0)
        {
            h_offset[i].x = acc;
            acc += h_offset[i].y;
        }
    }
    d_offset.copy_from(h_offset);
}

// -----------------------------------------------------------------------------

/// Allocate one more element at the top of the array to store another hrbf
/// instance
static void add_instance_memory()
{
    assert(!HRBF_env::binded);
    assert(hd_radius.device_array().size() == h_offset.size());

    assert(hd_radius. size() == h_offset.size());
    assert(hd_transfo.size() == h_offset.size());
    assert(d_offset.  size() == h_offset.size());

    const int size = h_offset.size() + 1;
    h_offset.  realloc( size );
    d_offset.  realloc( size );
    hd_radius. realloc( size );
    hd_transfo.realloc( size );

    h_offset[size - 1] = make_int2(0, 0);
    d_offset.set(size - 1, make_int2(0, 0));
    hd_radius [size - 1] = 5.f;
    hd_transfo[size - 1] = Transfo::identity();

    hd_radius. update_device_mem();
    hd_transfo.update_device_mem();
}

// -----------------------------------------------------------------------------

/// Erase an element at the top of the array
static void erase_instance_memory()
{
    assert(!HRBF_env::binded);
    assert(hd_radius.device_array().size() == h_offset.size());

    assert(hd_radius.size() == h_offset.size());
    assert(d_offset. size() == h_offset.size());

    h_offset.  realloc(h_offset.  size() - 1);
    d_offset.  realloc(d_offset.  size() - 1);
    hd_radius. realloc(hd_radius. size() - 1);
    hd_transfo.realloc(hd_transfo.size() - 1);

    hd_radius.update_device_mem();
    hd_transfo.update_device_mem();
}

// -----------------------------------------------------------------------------

int new_instance()
{
    assert(HRBF_env::binded);
    assert(nb_hrbf_instance >= 0);

    HRBF_env::unbind();

    // find the first free element
    int idx = 0;
    for(; idx<h_offset.size(); idx++)
        if( h_offset[idx].x < 0) break;

    // Add the instance of size 0
    if(idx == h_offset.size())
        add_instance_memory();

    update_offset(idx, 0);

    nb_hrbf_instance++;

    HRBF_env::bind();
    return idx;
}

// -----------------------------------------------------------------------------

void delete_instance(int hrbf_id)
{
    assert(HRBF_env::binded);
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert(nb_hrbf_instance > 0);
    HRBF_env::unbind();

    int inst_size = get_instance_size(hrbf_id);

    if(inst_size > 0)
    {
        int start = h_offset[hrbf_id].x;
        int end   = start + inst_size - 1;
        assert(start >= 0);
        d_init_points.    erase(start, end);
        d_init_alpha_beta.erase(start, end);
        d_map_transfos.   erase(start, end);
        h_normals.        erase(start, end);
        hd_points.        erase(start, end);
        hd_alphas_betas.  erase(start, end);

        hd_points.update_device_mem();
        hd_alphas_betas.update_device_mem();
    }

    // Compute the new offsets
    update_offset(hrbf_id, 0);

    if(hrbf_id == (h_offset.size()-1))
    {
        erase_instance_memory();
    }
    else
    {
        // Deleted hrbf instances are tag with negative offsets in order to
        // re-use the element for a new instance
        // the reason we do not simply erased the element hrbf_id is to avoid
        // changing the ids of the other instances which corresponds to an index
        // of h_offset
        h_offset[hrbf_id].x = -1;
    }

    nb_hrbf_instance--;

    HRBF_env::bind();
}

// -----------------------------------------------------------------------------

void reset_instance(int hrbf_id)
{
    assert(HRBF_env::binded);
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert(h_offset[hrbf_id].x >= 0 );
    assert(nb_hrbf_instance > 0);

    HRBF_env::unbind();
    float radius = hd_radius[hrbf_id];
    Transfo tr = get_transfo(hrbf_id);
    HRBF_env::bind();

    delete_instance(hrbf_id);

    HRBF_env::unbind();
    if(hrbf_id == h_offset.size())
        add_instance_memory();

    // Compute the new offsets
    update_offset(hrbf_id, 0);
    hd_radius.set_hd(hrbf_id, radius);
    set_transfo( hrbf_id, tr);
    nb_hrbf_instance++;
    HRBF_env::bind();
}

// -----------------------------------------------------------------------------

/// private function
/// Compute hrbf coeffs from d_points and h_normals. usefull when you
/// change/delete/add a sample.
/// the function is design to facilitate updates of HRBF_Env global variables
/// @warning don't forget to unbind array to textures before calling this
/// @param h_normals array in host memory
/// (parameter is likely to be HRBF_Env::h_normals.ptr()+offset)
/// @param d_points  array in device memory most likely
/// (parameter is likely to be HRBF_Env::d_init_points.ptr()+offset)
/// @param d_alphas_betas array in device memory
/// (parameter is likely to be HRBF_Env::d_init_alpha_beta.ptr()+offset)
/// @param nb_points size of the arrays
static void update_coeff(const Vec3_cu* h_normals,
                         const float4* d_points,
                         float4* d_alphas_betas,
                         int nb_points)
{
    assert(!HRBF_env::binded);
    using namespace HRBF_wrapper;

    HA_float4  vert_float(nb_points);
    HA_Vec3_cu vertices  (nb_points);
    HA_Vec3_cu normals   (nb_points);
    mem_cpy_hth(normals.ptr(), h_normals, nb_points);
    mem_cpy_dth(vert_float.ptr(), d_points, nb_points);
    for(int i=0; i<nb_points; i++){
        float4 v    = vert_float[i];
        vertices[i] = Vec3_cu(v.x, v.y, v.z);
    }

    HRBF_coeffs coeffs;
    hermite_fit(vertices.ptr(), normals.ptr(), nb_points, coeffs);
    // updates weights with the newly computed weights
    HA_float4 h_alpha_beta(nb_points);
    for(int i=0; i<nb_points; i++){
        Vec3_cu vBeta = coeffs.betas[i];
        h_alpha_beta[i] = make_float4(vBeta.x,vBeta.y,vBeta.z,coeffs.alphas[i]);
    }

    mem_cpy_htd(d_alphas_betas, h_alpha_beta.ptr(), nb_points );
}

// -----------------------------------------------------------------------------

static void update_anim_alpha_betas(int hrbf_id)
{
    const int offset = h_offset[hrbf_id].x;
    const int size   = get_instance_size(hrbf_id);

    mem_cpy_dth(hd_alphas_betas.ptr()+offset,
                d_init_alpha_beta.ptr()+offset,
                size);

    hd_alphas_betas.update_device_mem();
}

// -----------------------------------------------------------------------------

void set_sample(int hrbf_id,
                int sample_index,
                const Vec3_cu& p)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert( sample_index < get_instance_size(hrbf_id) );
    assert( sample_index >= 0 );

    HRBF_env::unbind();

    const int inst_size = get_instance_size(hrbf_id);
    const int offset    = h_offset[hrbf_id].x;
    assert(offset >= 0);
    float4 fpoint = make_float4(p.x, p.y, p.z, 1.f);
    int    idx    = sample_index+offset;
    d_init_points.set(idx, fpoint);
    hd_points.set_hd(idx, fpoint);
    // re-compute the weights
    update_coeff(h_normals.ptr()+offset,
                 d_init_points.ptr()+offset,
                 d_init_alpha_beta.ptr()+offset,
                 inst_size);

    update_anim_alpha_betas(hrbf_id);
    HRBF_env::bind();
}

// -----------------------------------------------------------------------------

void set_sample_normal(int hrbf_id,
                       int sample_index,
                       const Vec3_cu& n)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert( sample_index < get_instance_size(hrbf_id) );
    assert( sample_index >= 0 );

    HRBF_env::unbind();

    const int inst_size = get_instance_size(hrbf_id);
    const int offset    = h_offset[hrbf_id].x;
    assert(offset >= 0);
    int idx = sample_index+offset;
    h_normals[idx] = n;
    // re-compute the weights
    update_coeff(h_normals.ptr()+offset,
                 d_init_points.ptr()+offset,
                 d_init_alpha_beta.ptr()+offset,
                 inst_size);

    update_anim_alpha_betas(hrbf_id);

    HRBF_env::bind();
}

// -----------------------------------------------------------------------------

void set_inst_radius(int hrbf_id, float radius)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );

    HRBF_env::unbind();
    hd_radius.set_hd(hrbf_id, radius);
    HRBF_env::bind();
}

// -----------------------------------------------------------------------------

void set_transfo(int hrbf_id, const Transfo& tr)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert( hd_transfo.size() == h_offset.size());

    //HRBF_env::unbind();
    hd_transfo[hrbf_id] = tr;
    // will be done with apply_hrbf_transfos() :
    //hd_transfo.device_array().set(hrbf_id, tr);
    //HRBF_env::bind();
}

// -----------------------------------------------------------------------------

void apply_hrbf_transfos()
{
    hd_transfo.update_device_mem();
    HRBF_kernels::hrbf_transform(hd_transfo.device_array(), d_map_transfos);
}

// -----------------------------------------------------------------------------

void delete_samples(int hrbf_id, const std::vector<int>& samples_idx)
{
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert(HRBF_env::binded);

    HRBF_env::unbind();

    int size_inst = get_instance_size(hrbf_id);
    if( size_inst < 1 ){
        std::cerr << "There is no samples to delete";
        return;
    }

    // TODO:
    // delete samples, it's a prety ineficient way to do it because of the use
    // of erase() which duplicates the arrays each time it is called. But for
    // now its enough
    int offset = h_offset[hrbf_id].x;
    assert(offset >= 0);
    for(unsigned i=0; i<samples_idx.size(); i++)
    {
        int idx = samples_idx[i];
        d_init_points.    erase(idx + offset);
        d_init_alpha_beta.erase(idx + offset);
        d_map_transfos.   erase(idx + offset);
        h_normals.        erase(idx + offset);
        hd_points.        erase(idx + offset);
        hd_alphas_betas.  erase(idx + offset);
        hd_alphas_betas.update_device_mem();
        hd_points.      update_device_mem();
    }

    // Compute new offsets
    update_offset(hrbf_id, size_inst - samples_idx.size());

    update_coeff(h_normals.ptr()+offset,
                 d_init_points.ptr()+offset,
                 d_init_alpha_beta.ptr()+offset,
                 get_instance_size(hrbf_id));

    update_anim_alpha_betas(hrbf_id);

    HRBF_env::bind();
}

// -----------------------------------------------------------------------------

void delete_sample(int hrbf_id, int sample_idx)
{
    std::vector<int> idx(1);  idx[0] = sample_idx;
    delete_samples( hrbf_id, idx);
}

// -----------------------------------------------------------------------------

int add_samples(int hrbf_id,
                const std::vector<Vec3_cu>& points,
                const std::vector<Vec3_cu>& normals,
                const std::vector<float4>& weights)
{

    assert(points.size() == normals.size());
    assert(points.size() == weights.size()  || weights.size() == 0);
    assert(hrbf_id < h_offset.size());
    assert(hrbf_id >= 0);
    assert( h_offset[hrbf_id].x >= 0 );
    assert(HRBF_env::binded);

    if(points.size() == 0)
        return -1;

    HRBF_env::unbind();

    // add sample
    HA_float4 ha_points( points.size() );
    for(unsigned i = 0; i < points.size(); i++){
        Vec3_cu pt   = points[i];
        ha_points[i] = make_float4(pt.x, pt.y, pt.z, 1.f);
    }

    int inst_size = get_instance_size(hrbf_id);
    int offset    = h_offset[hrbf_id].x;
    assert(offset >= 0);
    d_init_points. insert(offset, ha_points);
    hd_points.     insert(offset, ha_points);
    h_normals.     insert(offset, normals  );
    d_map_transfos.insert(offset, std::vector<int>(points.size(), hrbf_id) );
    hd_points.update_device_mem();
    assert( h_normals.     size() == hd_points.    size() );
    assert( d_init_points. size() == hd_points.    size() );
    assert( d_map_transfos.size() == d_init_points.size() );

    // Compute new offsets
    update_offset(hrbf_id, inst_size+points.size());

    if(weights.size() == 0)
    {
        // We don't care what's in alpha_beta cause its gonna be erased by
        // update_coeff()
        d_init_alpha_beta.insert(offset, ha_points/*insert dummy data*/ );
        hd_alphas_betas  .insert(offset, ha_points/*insert dummy data*/ );

        update_coeff(h_normals.ptr()+offset,
                     d_init_points.ptr()+offset,
                     d_init_alpha_beta.ptr()+offset,
                     get_instance_size(hrbf_id));

        update_anim_alpha_betas(hrbf_id);
    }
    else
    {
        d_init_alpha_beta.insert(offset, weights );
        hd_alphas_betas  .insert(offset, weights );
        hd_alphas_betas.update_device_mem();
    }

    HRBF_env::bind();

    return get_instance_size(hrbf_id) - points.size();
}

// -----------------------------------------------------------------------------

int add_sample(int hrbf_id, const Vec3_cu& point, const Vec3_cu& normal)
{
    std::vector<Vec3_cu> points (1, point );
    std::vector<Vec3_cu> normals(1, normal);

    return add_samples(hrbf_id, points, normals);
}

// -----------------------------------------------------------------------------

}// END HRBF_ENV NAMESPACE =====================================================

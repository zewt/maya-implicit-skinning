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
#include "animesh.hpp"

#include "animesh_kers.hpp"
#include "macros.hpp"
#include "vec3_cu.hpp"
#include "distance_field.hpp"
#include "conversions.hpp"
#include "std_utils.hpp"
#include "skeleton.hpp"

// -----------------------------------------------------------------------------

#include <fstream>
#include <sstream>
#include <cstring>
#include <limits>
#include <cmath>

using namespace Cuda_utils;

namespace { __device__ void fix_debug() { } }

// -----------------------------------------------------------------------------

float distsqToSeg(const Point_cu& v, const Point_cu& p1, const Point_cu& p2);

// -----------------------------------------------------------------------------

Animesh::Animesh(const Mesh *m_, Skeleton* s_) :
    _mesh(m_), _skel(s_),
    mesh_smoothing(EAnimesh::LAPLACIAN),
    do_smooth_mesh(false),
    do_local_smoothing(true),
    do_interleave_fitting(false),
    do_update_potential(true),
    smoothing_iter(7),
    diffuse_smooth_weights_iter(6),
    smooth_force_a(0.5f),
    smooth_force_b(0.5f),
    smooth_smear(0.f),
    d_input_smooth_factors(_mesh->get_nb_vertices()),
    d_smooth_factors_conservative(_mesh->get_nb_vertices(), 0.f),
    d_smooth_factors_laplacian(_mesh->get_nb_vertices()),
    d_input_vertices(_mesh->get_nb_vertices()),
    d_edge_lengths(_mesh->get_nb_edges()),
    d_edge_mvc(_mesh->get_nb_edges()),
    d_vertices_state(_mesh->get_nb_vertices()),
    d_vertices_states_color(EAnimesh::NB_CASES),
//    d_input_normals(m->get_nb_vertices()),
    d_output_vertices(_mesh->get_nb_vertices()),
    d_output_normals(_mesh->get_nb_vertices()),
    d_ssd_vertices(_mesh->get_nb_vertices()),
    d_gradient(_mesh->get_nb_vertices()),
    d_input_tri(_mesh->get_nb_tri()*3),
    d_edge_list(_mesh->get_nb_edges()),
    d_edge_list_offsets(2 * _mesh->get_nb_vertices()),
    d_base_potential(_mesh->get_nb_vertices()),
    d_base_gradient(_mesh->get_nb_vertices()),
    d_piv(_mesh->get_nb_faces()),
    d_unpacked_normals(_mesh->get_nb_vertices() * _mesh->_max_faces_per_vertex),
    d_unpacked_tangents(_mesh->get_nb_vertices() * _mesh->_max_faces_per_vertex),
    d_rot_axis(_mesh->get_nb_vertices()),
    vertToBoneInfo(s_, m_),
    vmap_old_new(_mesh->get_nb_vertices()),
    vmap_new_old(_mesh->get_nb_vertices()),
    d_rear_verts(_mesh->get_nb_vertices()),
    h_half_angles(_skel->nb_joints()),
    d_half_angles(_skel->nb_joints()),
    h_orthos(_skel->nb_joints()),
    d_orthos(_skel->nb_joints()),
    h_vert_buffer(_mesh->get_nb_vertices()),
    d_vert_buffer(_mesh->get_nb_vertices()),
    d_vert_buffer_2(_mesh->get_nb_vertices()),
    d_vals_buffer(_mesh->get_nb_vertices())
{

    int nb_vert = _mesh->get_nb_vertices();
    Host::Array<EAnimesh::Vert_state> h_vert_state(nb_vert);
    for (int i = 0; i < nb_vert; ++i)
    {
        vmap_old_new[i] = i;
        vmap_new_old[i] = i;
        h_vert_state[i] = EAnimesh::NOT_DISPLACED;
    }

    d_vertices_state.copy_from(h_vert_state);

    d_vertices_states_color.set(EAnimesh::POTENTIAL_PIT      , make_float4(1.f, 0.f, 1.f, 0.99f)); // purple
    d_vertices_states_color.set(EAnimesh::GRADIENT_DIVERGENCE, make_float4(1.f, 0.f, 0.f, 0.99f)); // red
    d_vertices_states_color.set(EAnimesh::NB_ITER_MAX        , make_float4(0.f, 0.f, 1.f, 0.99f)); // blue
    d_vertices_states_color.set(EAnimesh::NOT_DISPLACED      , make_float4(1.f, 1.f, 0.f, 0.99f)); // yellow
    d_vertices_states_color.set(EAnimesh::FITTED             , make_float4(0.f, 1.f, 0.f, 0.99f)); // green
    d_vertices_states_color.set(EAnimesh::OUT_VERT           , make_float4(1.f, 1.f, 1.f, 0.99f)); // white
    d_vertices_states_color.set(EAnimesh::NORM_GRAD_NULL     , make_float4(0.f, 0.f, 0.f, 0.99f)); // black

    // Not mandatory but it is supposed to accelerate a little bit animation
    // when activated

    // Fill the attributes in device memory
    copy_mesh_data(*_mesh);

    set_default_bones_radius();

    init_smooth_factors(d_input_smooth_factors);
    init_vert_to_fit();

    compute_mvc();

    update_base_potential();
}

// -----------------------------------------------------------------------------

Animesh::~Animesh()
{
}

// -----------------------------------------------------------------------------

void Animesh::init_vert_to_fit()
{
    int nb_vert = _mesh->get_nb_vertices();
    std::vector<int> h_vert_to_fit_base;
    h_vert_to_fit_base.reserve(nb_vert);
    int acc = 0;
    for (int i = 0; i < nb_vert; ++i)
    {
        if( !_mesh->is_disconnect(i) ){
            h_vert_to_fit_base.push_back( i );
            acc++;
        }
    }

    d_vert_to_fit.     malloc(acc);
    d_vert_to_fit_base.malloc(acc);

    d_vert_to_fit_buff_scan.malloc(acc+1);
    d_vert_to_fit_buff.malloc(acc);
    h_vert_to_fit_buff.malloc(acc);
    h_vert_to_fit_buff_2.malloc(acc);

    d_vert_to_fit_base.copy_from(h_vert_to_fit_base);
    d_vert_to_fit.     copy_from(h_vert_to_fit_base);
}

// -----------------------------------------------------------------------------

void Animesh::copy_vertices(const std::vector<Vec3_cu> &vertices)
{
    assert(vertices.size() == d_input_vertices.size());
    const int nb_vert = vertices.size();
    Host::Array<Point_cu > input_vertices(nb_vert);

    for(int i = 0; i < nb_vert; i++)
    {
        Point_cu  pos = Convs::to_point(vertices[i]);

        input_vertices[i] = pos;
//        flip_prop     [i] = false; // XXX ?
    }

    d_input_vertices.copy_from(input_vertices);
}

void Animesh::copy_mesh_data(const Mesh& a_mesh)
{
    const int nb_vert = a_mesh.get_nb_vertices();

    Host::Array<Point_cu > input_vertices(nb_vert);
    for(int i = 0; i < nb_vert; i++)
    {
        Point_cu  pos = Convs::to_point( a_mesh.get_vertex(i) );
        input_vertices[i] = pos;
    }

    int n_faces = a_mesh.get_nb_faces();
    Host::Array<Mesh::PrimIdxVertices> h_piv(n_faces);
    for(int i = 0; i < n_faces; i++){
        h_piv[i] = a_mesh.get_piv(i);
    }
    d_piv.copy_from(h_piv);

    d_input_vertices.copy_from(input_vertices);

    HA_int h_edge_list(a_mesh.get_nb_edges());
    HA_int h_edge_list_offsets(2*nb_vert);
    for(int i = 0; i < a_mesh.get_nb_edges(); i++){
        h_edge_list[i] = a_mesh.get_edge(i);
    }
    for(int i = 0; i < nb_vert; i++){
        h_edge_list_offsets[2*i  ] = a_mesh.get_edge_offset(2*i  );
        h_edge_list_offsets[2*i+1] = a_mesh.get_edge_offset(2*i+1);
    }
    d_edge_list.copy_from(h_edge_list);
    d_edge_list_offsets.copy_from(h_edge_list_offsets);

    Cuda_utils::mem_cpy_htd(d_input_tri. ptr(), a_mesh.get_tri_index(), a_mesh.get_nb_tri()*3 );
}

// -----------------------------------------------------------------------------



void Animesh::compute_mvc()
{
    //Device::Array<Vec3_cu> d_grad( d_input_vertices.size() );
    Host::Array<float> edge_lengths(_mesh->get_nb_edges());
    Host::Array<float> edge_mvc    (_mesh->get_nb_edges());
    for(int i = 0; i < _mesh->get_nb_vertices(); i++)
    {
        Point_cu pos = Convs::to_point( _mesh->get_vertex(i)      );
        Vec3_cu  nor = Convs::to_point( _mesh->get_mean_normal(i) ); // FIXME : should be the gradient

        Mat3_cu frame = Mat3_cu::coordinate_system( nor ).transpose();
        float sum = 0.f;
        bool  out = false;
        // Look up neighborhood
        int dep      = _mesh->get_edge_offset(i*2    );
        int nb_neigh = _mesh->get_edge_offset(i*2 + 1);
        int end      = (dep+nb_neigh);

        if( nor.norm() < 0.00001f || _mesh->is_vert_on_side(i) ) {
            for(int n = dep; n < end; n++) edge_mvc[n] = 0.f;
        }
        else
        {
            for(int n = dep; n < end; n++)
            {
                int id_curr = _mesh->get_edge( n );
                int id_next = _mesh->get_edge( (n+1) >= end  ? dep   : n+1 );
                int id_prev = _mesh->get_edge( (n-1) <  dep  ? end-1 : n-1 );

                // compute edge length
                Point_cu  curr = Convs::to_point( _mesh->get_vertex(id_curr) );
                Vec3_cu e_curr = (curr - pos);
                edge_lengths[n] = e_curr.norm();

                // compute mean value coordinates
                // coordinates are computed by projecting the neighborhood to the
                // tangent plane
                {
                    // Project on tangent plane
                    Vec3_cu e_next = Convs::to_point( _mesh->get_vertex(id_next) ) - pos;
                    Vec3_cu e_prev = Convs::to_point( _mesh->get_vertex(id_prev) ) - pos;

                    e_curr = frame * e_curr;
                    e_next = frame * e_next;
                    e_prev = frame * e_prev;

                    e_curr.x = 0.f;
                    e_next.x = 0.f;
                    e_prev.x = 0.f;

                    float norm_curr_2D = e_curr.norm();

                    e_curr.normalize();
                    e_next.normalize();
                    e_prev.normalize();

                    // Computing mvc
                    float anext = std::atan2( -e_prev.z * e_curr.y + e_prev.y * e_curr.z, e_prev.dot(e_curr) );
                    float aprev = std::atan2( -e_curr.z * e_next.y + e_curr.y * e_next.z, e_curr.dot(e_next) );

                    float mvc = 0.f;
                    if(norm_curr_2D > 0.0001f)
                        mvc = (std::tan(anext*0.5f) + std::tan(aprev*0.5f)) / norm_curr_2D;

                    sum += mvc;
                    edge_mvc[n] = mvc;
                    out = out || mvc < 0.f;
                }
            }
            // we ignore points outside the convex hull
            if( sum  <= 0.f || out || isnan(sum) ) {
                for(int n = dep; n < end; n++) edge_mvc[n] = 0.f;
            }
        }

    }
    d_edge_lengths.copy_from( edge_lengths );
    d_edge_mvc.    copy_from( edge_mvc     );
}

void Animesh::get_default_junction_radius(std::vector<float> &nearest_rad) const
{
    const int nb_verts  = _mesh->get_nb_vertices();
    const int nb_joints = _skel->nb_joints();

    const float inf = std::numeric_limits<float>::infinity();
    nearest_rad = std::vector<float>(nb_joints, inf);

    // Junction radius is nearest vertex distance
    for(int i = 0; i < nb_verts; i++)
    {
        const int bone_id = vertToBoneInfo.h_vertices_nearest_bones[i];
        const Point_cu vert = _mesh -> get_vertex(i).to_point();
        float dist = _skel->get_bone(bone_id)->dist_to( vert );

        nearest_rad [bone_id] = std::min(nearest_rad [bone_id], dist);
    }

    for(int i = 0; i < nb_joints; i++)
    {
        if(nearest_rad[i] == inf)
            nearest_rad[i] = 1.f;
    }
}

void Animesh::set_default_bones_radius()
{
    const int nb_verts  = _mesh->get_nb_vertices();
    const int nb_joints = _skel->nb_joints();

    std::vector<float> avg_rad     (nb_joints);
    std::vector<float> farthest_rad(nb_joints);
    std::vector<int>   nb_smp      (nb_joints);

    const float inf = std::numeric_limits<float>::infinity();
    for(int i = 0; i < nb_joints; i++) {
        farthest_rad[i] = 0.f;
        avg_rad     [i] = 0.f;
        nb_smp      [i] = 0;
    }

    for(int i = 0; i < nb_verts; i++)
    {
        const int bone_id = vertToBoneInfo.h_vertices_nearest_bones[i];
        const Point_cu vert = _mesh -> get_vertex(i).to_point();
        float dist = _skel->get_bone(bone_id)->dist_to( vert );

        farthest_rad[bone_id] = std::max(farthest_rad[bone_id], dist);
        avg_rad[bone_id] += dist;
        nb_smp[bone_id]++;
    }

    for(int i = 0; i < nb_joints; i++)
    {
        // Cylinder radius is average vertices distance
        avg_rad[i] = nb_smp[i] ? avg_rad[i] / nb_smp[i] : 1.f;
        _skel->set_bone_radius(i, avg_rad[i]);

        // HRBF compact support radius is farthest vertex distance
        const float radius = farthest_rad[i] == 0.f ? 1.f : farthest_rad[i];
        _skel->set_bone_hrbf_radius(i, radius);
    }
}

// -----------------------------------------------------------------------------

float distsqToSeg(const Point_cu& v, const Point_cu& p1, const Point_cu& p2)
{
    Vec3_cu dir   = p2 - p1;
    Vec3_cu difp2 = p2 - v;

    if(difp2.dot(dir) < 0.f) return difp2.norm_squared();

    Vec3_cu difp1 = v - p1;
    float dot = difp1.dot(dir);

    if(dot <= 0.f) return difp1.norm_squared();

    return fmax(0.f, difp1.norm_squared() - dot*dot / dir.norm_squared());
}

// -----------------------------------------------------------------------------


Point_cu projToSeg(const Point_cu& v, const Point_cu& p1, const Point_cu& p2)
{

  Vec3_cu dir = p2 - p1;

  if( (p2 - v).dot(dir) < 0.f) return p2;

  float dot = (v - p1).dot(dir);

  if(dot <= 0.f) return p1;

  return p1 + dir * (dot / dir.norm_squared());
}

// -----------------------------------------------------------------------------

bool vectorInCone(const Vec3_cu& v, const std::vector<Vec3_cu>& ns)
{
    int i;
    Vec3_cu avg = Vec3_cu(0.f, 0.f, 0.f);
    for(i = 0; i < (int)ns.size(); ++i)
        avg += ns[i];

    return v.normalized().dot(avg.normalized()) > 0.5f;
}

void Animesh::init_smooth_factors(Cuda_utils::DA_float& d_smooth_factors)
{
    const int nb_vert = _mesh->get_nb_vertices();
    HA_float smooth_factors(nb_vert);

    for(int i=0; i<nb_vert; i++)
        smooth_factors[i] = 0.0f;

    d_smooth_factors.copy_from(smooth_factors);
}

// -----------------------------------------------------------------------------

void Animesh::diffuse_attr(int nb_iter, float strength, float *attr)
{
    Animesh_kers::diffuse_values(attr,
                            d_vals_buffer.ptr(),
                            d_edge_list,
                            d_edge_list_offsets,
                            strength,
                            nb_iter);
}

// -----------------------------------------------------------------------------

void Animesh::get_anim_vertices_aifo(std::vector<Point_cu>& anim_vert)
{
    const int nb_vert = d_output_vertices.size();
    anim_vert.reserve(nb_vert);
    Cuda_utils::HA_Point_cu h_out_verts(nb_vert);
    h_out_verts.copy_from(d_output_vertices);

    for(int i = 0; i < nb_vert; i++)
        anim_vert.push_back(h_out_verts[vmap_new_old[i]]);
}

// -----------------------------------------------------------------------------

void Animesh::set_bone_type(int id, int bone_type)
{
    // Don't waste memory converting joints with no associated vertices.
    // XXX: but we convert to HRBF elsewhere and we can't leave bones in HRBF, even if
    // they're empty (eg. bbox will be slow)
//    if(bone_type != EBone::SSD && h_verts_id_per_bone[id].size() == 0)
//        return;

    // Make sure that transform_hrbf/transform_precomputed_prim has been
    // called.  XXX: This probably shouldn't be needed here, or at least
    // we could only update the correct bone so we don't do n^2 updates.
    _skel->update_bones_pose();

    Bone *bone = _skel->get_bone( id );
    switch(bone_type){
    case EBone::PRECOMPUTED:
    {
        bone->set_enabled(true);
        bone->precompute(_skel->get_skel_id());
        break;
    }
    case EBone::HRBF:
        bone->set_enabled(true);
        bone->discard_precompute();
        break;
    case EBone::SSD:
        bone->set_enabled(false);
        break;

    default: //unknown bone type !
        assert(false);
        break;

    }

    init_vert_to_fit();

    // XXX: It makes sense that we need this here, but if we don't call it we hang in a weird way.
    // Figure out why for diagnostics.
    _skel->update_bones_pose();
}

int Animesh::pack_vert_to_fit(Cuda_utils::Host::Array<int>& in,
                                   Cuda_utils::Host::Array<int>& out,
                                   int size)
{
    Cuda_utils::mem_cpy_dth(in.ptr(), d_vert_to_fit.ptr(), size);

    int j = 0;
    for(int i = 0; i < size; i++)
    {
        int elt = in[i];
        if(elt != -1)
        {
            out[j] = elt;
            j++;
        }
    }

    Cuda_utils::mem_cpy_htd(d_vert_to_fit.ptr(), out.ptr(), j);
    return j;
}

// -----------------------------------------------------------------------------

#include "cuda_utils_thrust.hpp"

__global__ static
void transform_vert_to_fit(const int* src, int* dst, const int nb_vert)
{
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_vert) dst[p] = src[p] < 0 ? 0 : 1;
}

/// here src must be different from dst
__global__ static
void pack(const int* prefix_sum, const int* src, int* dst, const int nb_vert)
{
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_vert){
        const int elt = src[p];
        if(elt >= 0) dst[ prefix_sum[p] ] = elt;
    }
}

int Animesh::pack_vert_to_fit_gpu(
        Cuda_utils::Device::Array<int>& d_vert_to_fit,
        Cuda_utils::Device::Array<int>& buff,
        Cuda_utils::Device::Array<int>& packed_array,
        int nb_vert_to_fit)
{
    if(nb_vert_to_fit == 0) return 0;
    assert(d_vert_to_fit.size() >= nb_vert_to_fit           );
    assert(buff.size()          >= d_vert_to_fit.size() + 1 );
    assert(packed_array.size()  >= d_vert_to_fit.size()     );

    const int block_s = 16;
    const int grid_s  = (nb_vert_to_fit + block_s - 1) / block_s;
    transform_vert_to_fit<<<grid_s, block_s >>>(d_vert_to_fit.ptr(), buff.ptr()+1, nb_vert_to_fit);
    buff.set(0, 0);// First element to zero
    CUDA_CHECK_ERRORS();

    // Compute prefix sum in buff between [1 nb_vert_to_fit]
    Cuda_utils::inclusive_scan(0, nb_vert_to_fit-1, buff.ptr()+1);

    const int new_nb_vert_to_fit = buff.fetch(nb_vert_to_fit);

    pack<<<grid_s, block_s >>>(buff.ptr(), d_vert_to_fit.ptr(), packed_array.ptr(), nb_vert_to_fit);
    CUDA_CHECK_ERRORS();

    //Cuda_utils::mem_cpy_dtd(d_vert_to_fit.ptr(), packed_array.ptr(), new_nb_vert_to_fit);

    return new_nb_vert_to_fit;
}

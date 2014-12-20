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

#include "animesh_kers_colors.hpp"
#include "animesh_kers.hpp"
#include "macros.hpp"
#include "vec3_cu.hpp"
#include "distance_field.hpp"
#include "auto_rig.hpp"
#include "color_ctrl.hpp"
#include "conversions.hpp"
#include "glbuffer_object.hpp"
#include "std_utils.hpp"
#include "skeleton.hpp"

// -----------------------------------------------------------------------------

#include <fstream>
#include <sstream>
#include <cstring>
#include <limits>
#include <cmath>

// -----------------------------------------------------------------------------

namespace Cuda_ctrl{
    extern Color_ctrl _color;
}

using namespace Cuda_utils;

// -----------------------------------------------------------------------------

float distsqToSeg(const Point_cu& v, const Point_cu& p1, const Point_cu& p2);

// -----------------------------------------------------------------------------

Animesh::Animesh(Mesh* m_, Skeleton* s_) :
    _mesh(m_), _skel(s_),
    _do_bone_deform(s_->nb_joints(), true),
    mesh_color(EAnimesh::BASE_POTENTIAL),
    mesh_smoothing(EAnimesh::LAPLACIAN),
    do_implicit_skinning(false),
    do_smooth_mesh(false),
    do_local_smoothing(true),
    do_interleave_fitting(false),
    do_update_potential(true),
    smoothing_iter(7),
    diffuse_smooth_weights_iter(6),
    smooth_force_a(0.5f),
    smooth_force_b(0.5f),
    smooth_smear(0.f),
    _vox_mesh(0),
    d_input_smooth_factors(_mesh->get_nb_vertices()),
    d_smooth_factors_conservative(_mesh->get_nb_vertices(), 0.f),
    d_smooth_factors_laplacian(_mesh->get_nb_vertices()),
    d_input_vertices(_mesh->get_nb_vertices()),
    d_edge_lengths(_mesh->get_nb_edges()),
    d_edge_mvc(_mesh->get_nb_edges()),
    d_vertices_state(_mesh->get_nb_vertices()),
    d_vertices_states_color(EAnimesh::NB_CASES),
    h_junction_radius(_skel->nb_joints()),
//    d_input_normals(m->get_nb_vertices()),
    d_output_vertices(_mesh->get_nb_vertices()),
    d_output_normals(_mesh->get_nb_vertices()),
    d_output_tangents(_mesh->get_nb_vertices()),
    d_ssd_normals(_mesh->get_nb_vertices()),
    d_ssd_vertices(_mesh->get_nb_vertices()),
    d_gradient(_mesh->get_nb_vertices()),
    d_input_tri(_mesh->get_nb_tri()*3),
    d_input_quad(_mesh->get_nb_quad()*4),
    d_edge_list(_mesh->get_nb_edges()),
    d_edge_list_offsets(2 * _mesh->get_nb_vertices()),
    d_joints(), d_weights(),
    d_jpv(2 * _mesh->get_nb_vertices()),
    h_weights(_mesh->get_nb_vertices()),
    d_base_potential(_mesh->get_nb_vertices()),
    d_base_gradient(_mesh->get_nb_vertices()),
    d_piv(_mesh->get_nb_faces()),
    d_packed_vert_map(_mesh->get_nb_vertices()),
    d_unpacked_normals(_mesh->get_nb_vertices() * _mesh->_max_faces_per_vertex),
    d_unpacked_tangents(_mesh->get_nb_vertices() * _mesh->_max_faces_per_vertex),
    d_rot_axis(_mesh->get_nb_vertices()),
    d_ssd_interpolation_factor(_mesh->get_nb_vertices(), 0.f),
    h_vertices_nearest_bones(_mesh->get_nb_vertices()),
    d_vertices_nearest_bones(_mesh->get_nb_vertices()),
    nb_vertices_by_bones(_skel->get_bones().size()),
    h_vertices_nearest_joint(_mesh->get_nb_vertices()),
    d_vertices_nearest_joint(_mesh->get_nb_vertices()),
    d_nearest_bone_in_device_mem(_mesh->get_nb_vertices()),
    d_nearest_joint_in_device_mem(_mesh->get_nb_vertices()),
    h_nearest_bone_dist(_mesh->get_nb_vertices()),
    vmap_old_new(_mesh->get_nb_vertices()),
    vmap_new_old(_mesh->get_nb_vertices()),
    d_rear_verts(_mesh->get_nb_vertices()),
    h_half_angles(_skel->nb_joints()),
    d_half_angles(_skel->nb_joints()),
    h_orthos(_skel->nb_joints()),
    d_orthos(_skel->nb_joints()),
    d_flip_propagation(_mesh->get_nb_vertices()),
    h_vert_buffer(_mesh->get_nb_vertices()),
    d_vert_buffer(_mesh->get_nb_vertices()),
    d_vert_buffer_2(_mesh->get_nb_vertices()),
    d_vals_buffer(_mesh->get_nb_vertices())
{

    ////////////////////////////////////////////////////////////////////////////////// HACK: DEBUG:
    //_do_bone_deform[_skel->root()] = false;
    ////////////////////////////////////////////////////////////////////////////////// HACK: DEBUG:

    // Compute nearest bone and nearest joint from each vertices
    clusterize( EAnimesh::EUCLIDEAN );

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
#if 0
    // Modified mesh in order to vertices to be ordered by bone cluster
    regroup_mesh_vertices(*m,
                          d_vertices_nearest_bones,
                          h_vertices_nearest_bones,
                          d_vertices_nearest_joint,
                          h_vertices_nearest_joint,
                          vmap_old_new);

    Color cl = Cuda_ctrl::_color.get(Color_ctrl::MESH_POINTS);
    m->set_point_color_bo(cl.r, cl.g, cl.b, cl.a);
#endif

    // Fill the attributes in device memory
    copy_mesh_data(*_mesh);

    set_default_bones_radius();

    init_smooth_factors(d_input_smooth_factors);
    init_rigid_ssd_weights();
    init_ssd_interpolation_weights();

    compute_mvc();

    update_base_potential();
    set_colors(mesh_color);
}

// -----------------------------------------------------------------------------

void Animesh::init_verts_per_bone()
{
    std::vector< std::vector<Vec3_cu> >& vertices = h_input_verts_per_bone;
    std::vector< std::vector<Vec3_cu> >& normals  = h_input_normals_per_bone;
    std::vector< std::vector<int>     >& vert_ids = h_verts_id_per_bone;
    vertices.clear();
    normals. clear();
    vertices.resize(_skel->nb_joints());
    vert_ids.resize(_skel->nb_joints());
    normals. resize(_skel->nb_joints());

    for(int i = 0; i < _mesh->get_nb_vertices(); i++)
    {
        int nearest = h_vertices_nearest_bones[i];

        if(_mesh->is_disconnect(i))
            continue;

        const Vec3_cu vert = _mesh->get_vertex(i);
        const Vec3_cu norm = _mesh->get_normal(i);

        vertices[nearest].push_back( Vec3_cu(vert.x,  vert.y,  vert.z)              );
        normals [nearest].push_back( Vec3_cu(norm.x,  norm.y,  norm.z).normalized() );

        vert_ids[nearest].push_back( i );
    }
}

// -----------------------------------------------------------------------------

Animesh::~Animesh()
{
}

// -----------------------------------------------------------------------------

void Animesh::init_vert_to_fit()
{
    assert(d_ssd_interpolation_factor.size() > 0);

    Cuda_utils::HA_float ssd_factor(d_ssd_interpolation_factor.size());
    ssd_factor.copy_from(d_ssd_interpolation_factor);

    int nb_vert = _mesh->get_nb_vertices();
    std::vector<int> h_vert_to_fit_base;
    h_vert_to_fit_base.reserve(nb_vert);
    int acc = 0;
    for (int i = 0; i < nb_vert; ++i)
    {
        if( !_mesh->is_disconnect(i) && ssd_factor[i] < (1.f - 0.00001f) ){
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

void Animesh::copy_mesh_data(const Mesh& a_mesh)
{
    const int nb_vert = a_mesh.get_nb_vertices();

    const Mesh::Packed_data* d = a_mesh.get_packed_vert_map();
    Cuda_utils::mem_cpy_htd(d_packed_vert_map.ptr(), d, nb_vert);
    if(a_mesh._has_tex_coords)
    {
        Vec3_cu* t = (Vec3_cu*)a_mesh._tangents;
        Cuda_utils::mem_cpy_htd(d_output_tangents.ptr(), t, nb_vert);
    }

    Host::Array<Point_cu > input_vertices(nb_vert);
    Host::Array<Vec3_cu>   input_normals (nb_vert);
    Host::Array<bool>      flip_prop     (nb_vert);
    for(int i = 0; i < nb_vert; i++)
    {
        Point_cu  pos = Convs::to_point( a_mesh.get_vertex(i) );

        input_vertices[i] = pos;
        input_normals [i] = a_mesh.get_normal(i);
        flip_prop     [i] = false;
    }

    int n_faces = a_mesh.get_nb_faces();
    Host::Array<Mesh::PrimIdxVertices> h_piv(n_faces);
    for(int i = 0; i < n_faces; i++){
        h_piv[i] = a_mesh.get_piv(i);
    }
    d_piv.copy_from(h_piv);

    d_input_vertices.copy_from(input_vertices);
//    d_input_normals.copy_from(input_normals);
    d_ssd_normals.copy_from(input_normals);
    d_flip_propagation.copy_from(flip_prop);

    _vbo_input_vert   = new GlBuffer_obj( _mesh->get_vbos_size()*3, GL_ARRAY_BUFFER, GL_STATIC_DRAW);
    _nbo_input_normal = new GlBuffer_obj( _mesh->get_vbos_size()*3, GL_ARRAY_BUFFER, GL_STATIC_DRAW);

    _vbo_input_vert->cuda_register();
    _nbo_input_normal->cuda_register();

    update_opengl_buffers(nb_vert, (Vec3_cu*)d_input_vertices.ptr(), (Vec3_cu*)d_ssd_normals.ptr(), 0,
                          _vbo_input_vert, _nbo_input_normal, 0);

    _vbo_input_vert->cuda_unregister();
    _nbo_input_normal->cuda_unregister();

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

    Cuda_utils::mem_cpy_htd(d_input_tri. ptr(), a_mesh._tri , a_mesh._nb_tri*3 );
    Cuda_utils::mem_cpy_htd(d_input_quad.ptr(), a_mesh._quad, a_mesh._nb_quad*4);
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

// -----------------------------------------------------------------------------
#if 0
static bool is_cluster_ssd(const Skeleton* s, int id)
{
    bool ssd = s->bone_type( id ) == Bone_type::SSD;
    const int pt = s->parent( id );
    if( pt > -1 )
    {
        const std::vector<int>& sons = s->get_sons( pt );
        for (unsigned j = 0; j < sons.size(); ++j)
            ssd = ssd && (s->bone_type( sons[j] ) == Bone_type::SSD);
    }

    return ssd;
}
#endif

// -----------------------------------------------------------------------------

void Animesh::init_ssd_interpolation_weights()
{
    int n = d_input_vertices.size();

//    Host::Array<float> base_potential(n);
//    base_potential.copy_from(d_base_potential);

    Host::Array<float> base_ssd_weights(n);
    base_ssd_weights.copy_from(d_ssd_interpolation_factor);
#if 0
    for(int i = 0; i < n; i++)
    {
        base_ssd_weights[i] = 0.f;
#if 1
        const Point_cu vert = _mesh->get_vertex(i).to_point();
        const int nearest = h_vertices_nearest_bones[i];
        Bone_cu b = _skel->get_bone_rest_pose( nearest );
        const float len = b.length() > 0.0001f ? b.length() : 1.f;

        bool ssd = is_cluster_ssd(_skel, nearest);

        bool fit;
        if( ssd )
            fit = false;
         else
        {
            // cluster parent is ssd ?
            int pt = _skel->get_parent( nearest );
            bool pt_ssd = ( pt > -1 && is_cluster_ssd(_skel, pt));

            // cluster son is ssd
            const std::vector<int>& sons = _skel->get_sons( nearest );
            bool s_ssd = ( sons.size() > 0 &&
                           is_cluster_ssd(_skel, sons[0]) &&
                           !_skel->is_leaf( sons[0]));

            if( (b.dist_proj_to( vert )/len) < 0.5f )
                fit = !pt_ssd; // Don't fit if parent cluster ssd
            else
                fit = !s_ssd; // Don't fit if son cluster ssd

        }
        base_ssd_weights[i] = fit ? 0.f : 1.f;
#endif
//        if(base_potential[i] <= 0.f) base_ssd_weights[i] = 1.f;
//        else                         base_ssd_weights[i] = 0.f;

    }
#endif
    //_mesh->diffuse_along_mesh(base_ssd_weights.ptr(), 1.f, 2);

    d_ssd_interpolation_factor.copy_from(base_ssd_weights);

    init_vert_to_fit();

    if( mesh_color == EAnimesh::SSD_INTERPOLATION)
        set_colors( mesh_color );
}

// -----------------------------------------------------------------------------

void Animesh::export_off(const char* filename) const
{

    // FIXME: take into acount quads
    // FIXME: use vmap_old_new to write index and vertices in the old order
    assert(false);

    using namespace std;
    ofstream file(filename);

    if(!file.is_open()){
        cerr << "Error exporting file " << filename << endl;
        exit(1);
    }

    file << "OFF" << endl;
    file << _mesh -> get_nb_vertices() << ' ' << _mesh -> get_nb_faces() << " 0" << endl;
    Vec3_cu* output_vertices;
    _mesh->_vbo.cuda_map_to(output_vertices);
    float* vertices = new float[3 * _mesh -> get_nb_vertices()];
    CUDA_SAFE_CALL(cudaMemcpy(vertices,
                              output_vertices,
                              3 * _mesh -> get_nb_vertices() * sizeof(float),
                              cudaMemcpyDeviceToHost));
    _mesh->_vbo.cuda_unmap();
    for(int i = 0; i < _mesh -> get_nb_vertices(); i++){
        file << vertices[3 * i] << ' '
             << vertices[3 * i + 1] << ' '
             << vertices[3 * i + 2] << ' ' << endl;
    }

    for(int i = 0; i < _mesh -> get_nb_faces(); i++) {
        int a = _mesh -> get_tri(3*i);
        int b = _mesh -> get_tri(3*i+1);
        int c = _mesh -> get_tri(3*i+2);
        file << "3 " << a << ' ' << b << ' ' << c << endl;
    }

    file.close();
}

// -----------------------------------------------------------------------------

void Animesh::draw(bool use_color_array, bool use_point_color) const
{
    if(!use_point_color)
        _mesh->draw_using_buffer_object( _mesh->_vbo, _mesh->_normals_bo, use_color_array );
    else
        _mesh->draw_using_buffer_object( _mesh->_vbo, _mesh->_normals_bo, _mesh->_point_color_bo, use_color_array );
}

// -----------------------------------------------------------------------------

void Animesh::draw_rest_pose(bool use_color_array, bool use_point_color) const
{
    if(!use_point_color)
        _mesh->draw_using_buffer_object(*_vbo_input_vert, *_nbo_input_normal, use_color_array );
    else
        _mesh->draw_using_buffer_object(*_vbo_input_vert, *_nbo_input_normal, _mesh->_point_color_bo, use_color_array );
}

// -----------------------------------------------------------------------------

void Animesh::draw_points_rest_pose() const
{
    _mesh->draw_points_using_buffer_object(*_vbo_input_vert, *_nbo_input_normal, _mesh->_point_color_bo, true);
}


// -----------------------------------------------------------------------------

float Animesh::compute_nearest_vert_to_bone(int bone_id)
{
    return 1.f;
}

// -----------------------------------------------------------------------------

void Animesh::clusterize_euclidean(HA_int& vertices_nearest_bones,
                                   HA_int& h_vertices_nearest_joint,
                                   HA_int& nb_vert_by_bone)
{
    const int nb_bones = _skel->get_bones().size();
    assert(nb_vert_by_bone.size() == nb_bones);
    for(int i = 0; i<nb_bones; i++)
        nb_vert_by_bone[i] = 0;

    int n = _mesh->get_nb_vertices();
    for(int i = 0; i < n ; i++)
    {
        float d0  = std::numeric_limits<float>::infinity();
        int   nd0 = _skel->root();

        float joint_dist       = std::numeric_limits<float>::infinity();
        int   nearest_joint_id = _skel->root();

        const Point_cu current_vertex = _mesh->get_vertex(i).to_point();
        for(int j = 0; j < _skel->nb_joints(); j++)
        {
            const Bone* b = _skel->get_bone( j );

            if( _skel->is_leaf(j) || !_do_bone_deform[j] )
                continue;

            // Compute nearest bone
            float dist2 = b->dist_sq_to(current_vertex);

            if(dist2 <= d0){
                d0  = dist2;
                nd0 = j;
            }

            // compute nearest joint
            const Point_cu joint = _skel->joint_pos(j).to_point();
            const Vec3_cu dir    = current_vertex-joint;
            float dist = dir.norm();
            // works fine but some mesh have corrupted normals so for the moment
            // I don't use this information
            float sign = 1.f;// dir.dot( current_normal );
            if(dist < joint_dist && sign >= 0){
                nearest_joint_id = j;
                joint_dist       = dist;
            }
        }
        h_nearest_bone_dist     [i] = sqrt(d0);
        vertices_nearest_bones  [i] = nd0;
        h_vertices_nearest_joint[i] = nearest_joint_id;
        nb_vert_by_bone[nd0]++;
    }
}

// -----------------------------------------------------------------------------

void Animesh::regroup_mesh_vertices(Mesh& a_mesh,
                                    DA_int& d_nearest_bones,
                                    HA_int& nearest_bones,
                                    DA_int& d_nearest_joints,
                                    HA_int& nearest_joints,
                                    HA_int& vmap_old_new)
{
    assert(false);

}

// -----------------------------------------------------------------------------

void Animesh::clusterize(EAnimesh::Cluster_type type, int n_voxels)
{
    if( type == EAnimesh::EUCLIDEAN )
    {
        clusterize_euclidean(h_vertices_nearest_bones,
                             h_vertices_nearest_joint,
                             nb_vertices_by_bones);
    }
    else
    {
        /*
        clusterize_geodesic(h_vertices_nearest_bones,
                            h_vertices_nearest_joint,
                            nb_vertices_by_bones,
                            n_voxels);
                            */

    }

    init_verts_per_bone();
    update_nearest_bone_joint_in_device_mem();
}

// -----------------------------------------------------------------------------

void Animesh::update_nearest_bone_joint_in_device_mem()
{
    int n = _mesh->get_nb_vertices();
    // Convert host ids to device ids for the nearest joints
    std::vector<DBone_id> tmp (n);
    std::vector<DBone_id> tmp2(n);
    for(int i = 0; i < n; i++){
        tmp [i] = _skel->get_bone_didx( h_vertices_nearest_bones[i] );
        tmp2[i] = _skel->get_bone_didx( h_vertices_nearest_joint[i] );
    }
    d_nearest_bone_in_device_mem. copy_from(tmp);
    d_nearest_joint_in_device_mem.copy_from(tmp2);

    d_vertices_nearest_bones.copy_from(h_vertices_nearest_bones);
    d_vertices_nearest_joint.copy_from(h_vertices_nearest_joint);
}

// -----------------------------------------------------------------------------

void Animesh::set_default_bones_radius()
{
    const int nb_verts  = _mesh->get_nb_vertices();
    const int nb_joints = _skel->nb_joints();

    std::vector<float> avg_rad     (nb_joints);
    std::vector<float> nearest_rad (nb_joints);
    std::vector<float> farthest_rad(nb_joints);
    std::vector<int>   nb_smp      (nb_joints);

    const float inf = std::numeric_limits<float>::infinity();
    for(int i = 0; i < nb_joints; i++) {
        nearest_rad [i] = inf;
        farthest_rad[i] = 0.f;
        avg_rad     [i] = 0.f;
        nb_smp      [i] = 0;
    }

    for(int i = 0; i < nb_verts; i++)
    {
        const int j = h_vertices_nearest_bones[i];
        const Point_cu vert = _mesh -> get_vertex(i).to_point();
        float  d = _skel->get_bone(j)->dist_to( vert );

        nearest_rad [j] = std::min(nearest_rad [j], d);
        farthest_rad[j] = std::max(farthest_rad[j], d);
        avg_rad[j] += d;
        nb_smp[j]++;
    }

    for(int i = 0; i < nb_joints; i++)
    {
        // Cylinder radius is average vertices distance
        avg_rad[i] = nb_smp[i] ? avg_rad[i] / nb_smp[i] : 1.f;
        _skel->set_bone_radius(i, avg_rad[i]);

        // HRBF compact support radius is farthest vertex distance
        const float radius = farthest_rad[i] == 0.f ? 1.f : farthest_rad[i];
        _skel->set_bone_hrbf_radius(i, radius);

        // Junction radius is nearest vertex distance
        h_junction_radius[i] = nearest_rad[i] == inf ? 1.f : nearest_rad[i];
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

// -----------------------------------------------------------------------------

void Animesh::heat_diffuse_ssd_weights(float heat)
{

#if 1
    // Heat diffusion code :
//    assert(_mesh->is_closed());
//    assert(_mesh->is_manifold());
    assert(_mesh->_nb_quad == 0);

    init_rigid_ssd_weights();

    std::cout << "heat coeff" << heat << std::endl;
    int nv       = _mesh->get_nb_vertices();
    int nb_bones = _skel->nb_joints();

    std::vector< Vec3_cu             > vertices(nv);
    // edges[nb_vertex][nb_vertex_connected_to_it]
    std::vector< std::vector<int>    > edges(nv);
    std::vector< std::vector<double> > boneDists(nv);
    std::vector< std::vector<bool>   > boneVis(nv);
    std::vector<std::vector<std::pair<int, double> > > nzweights(nv);

    // Fill edges and vertices
    for(int i = 0; i<nv; ++i)
    {
        Vec3_cu v = _mesh->get_vertex(i);
        vertices[i] = Vec3_cu(v.x, v.y, v.z);
        int dep = _mesh->get_edge_offset(i*2    );
        int off = _mesh->get_edge_offset(i*2 + 1);
        for(int j=dep; j< (dep+off); ++j )
            edges[i].push_back(_mesh->get_edge(j));
    }

    // Compute bone visibility and distance from a vertex to each bone
    for(int i = 0; i < nv; ++i)
    {
        boneDists[i].resize(nb_bones, -1);
        boneVis[i].resize(nb_bones, true);
        Point_cu cPos = Convs::to_point( vertices[i] );

        // Compute the normal of each adjacent face
        std::vector<Vec3_cu> normals;
        for(int j = 0; j < (int)edges[i].size(); ++j)
        {
            int nj = (j + 1) % edges[i].size();
            Vec3_cu v1 = vertices[edges[i][j] ] - cPos;
            Vec3_cu v2 = vertices[edges[i][nj]] - cPos;
            Vec3_cu n  = (v1.cross(v2)).normalized();
            normals.push_back( n );
        }

#if 1
        // Compute the nearest bone euclidean distance to the current vertex

        const double inf = std::numeric_limits<double>::infinity();;
        double minDist   = inf;
        for(int j = 0; j < nb_bones; ++j)
        {
            if( _skel->is_leaf(j) ){
                boneDists[i][j] = inf;
                continue;
            }

            Bone_cu bone = _skel->get_bone_rest_pose(j);

            Point_cu p1 = bone._org;
            Point_cu p2 = bone._org + bone._dir;
            boneDists[i][j] = sqrt(distsqToSeg(cPos, p1, p2));
            minDist = min(boneDists[i][j], minDist);
        }
#endif

        for(int j = 0; j < nb_bones; ++j)
        {
            //the reason we don't just pick the closest bone is so that if two are
            //equally close, both are factored in.
            if(boneDists[i][j] > minDist * 1.0001)
                continue;

            //Bone_cu bone = _skel->get_bone_rest_pose( j );

            //Point_cu pos  = cPos;
            //Point_cu p1   = bone._org;
            //Point_cu p2   = bone._org + bone._dir;
            //Point_cu proj = projToSeg(pos, p1, p2);

            if( _skel->is_leaf(j) )
                boneVis[i][j] = false;
            else
                // TODO: implement visibility test
                boneVis[i][j] = /*tester->canSee(cPos, p)*/true /*&& vectorInCone(-(pos - proj), normals)*/;
        }
    }

#if 0
    // Fill boneDists with voxel distance
    for(int j = 0; j < nb_bones; ++j)
    {
        std::vector<Vox::Segment> bones;
        Bone_cu bone = _skel->get_bone_rest_pose(j+1);

        Point_cu org = bone._org;
        Point_cu end = bone._org + bone->_dir;
        bones.push_back( Vox::Segment(Vox::V3d(org.x, org.y, org.z),
                                      Vox::V3d(end.x, end.y, end.z) )
                        );

        Grid grid_closest = Vox::Geo_dist::closest_bone(*_voxels, bones);

        for(int i = 0; i < nv; ++i)
        {
            Vec3_cu    v     = _mesh->get_vertex(i);
            Vox::V3d  x     = Vox::V3d(v.x, v.y, v.z);
            Vox::V3d  x_vox = grid_closest.convert_real_to_voxel_index( x );
            Vox::Idx3 u_vox = x_vox.to_index_3();
            Vox::Geo_dist::Cell c = grid_closest.get( u_vox );
            boneDists[i][j] = c._dist;
        }
    }
#endif

    // Compute weights
    rig(vertices, edges, boneDists, boneVis, nzweights, heat,  h_vertices_nearest_bones.ptr(), h_nearest_bone_dist);

    // convert nzweights to our array representation
    std::vector<float> h_weights; h_weights.reserve(2 * nv);
    std::vector<int>   h_joints; h_joints.reserve( 2 * nv);
    Host::Array<int>   h_jpv(2u*nv);

    int acc = 0;
    for(int i=0; i<nv; ++i)
    {
        int size = nzweights[i].size();
        for(int j=0; j<size; j++)
        {
            int   joint_id = nzweights[i][j].first;
            float weight   = (float)nzweights[i][j].second;

            h_joints. push_back( joint_id );
            h_weights.push_back( weight   );
        }
        h_jpv[i*2    ] = acc;  // starting index
        h_jpv[i*2 + 1] = size; // number of bones influencing the vertex
        acc += size;
    }

    d_jpv.copy_from(h_jpv);
    d_weights.malloc(acc);
    d_weights.copy_from(h_weights);
    d_joints.malloc(acc);
    d_joints.copy_from(h_joints);

    update_host_ssd_weights();
#endif
}

// -----------------------------------------------------------------------------

void Animesh::init_rigid_ssd_weights()
{
    int nb_vert = _mesh->get_nb_vertices();

    std::vector<float> weights(nb_vert);
    std::vector<int>   joints (nb_vert);
    Host::Array<int>   jpv    (2u*nb_vert);

    for(int i = 0; i < nb_vert; ++i)
    {
        joints [i] = h_vertices_nearest_bones[i];
        weights[i] = 1.f;

        jpv[i*2    ] = i; // starting index
        jpv[i*2 + 1] = 1; // number of bones influencing the vertex

        int start = i;
        int end   = start + 1;

        h_weights[i].clear();
        for(int j = start; j < end ; j++)
            h_weights[i][joints[j]] = weights[j];
    }

    d_jpv.copy_from(jpv);
    d_weights.malloc(nb_vert);
    d_weights.copy_from(weights);
    d_joints.malloc(nb_vert);
    d_joints.copy_from(joints);
}

// -----------------------------------------------------------------------------

void Animesh::geodesic_diffuse_ssd_weights(int nb_iter, float strength)
{
    std::vector<std::map<int, float> >& weights = h_weights;

    // diffuse weights
    std::vector<std::map<int, float> > new_weights(weights.size());
    std::map<int, float>::iterator it;
    for(int iter = 0; iter < nb_iter; iter++)
    {
        // For all vertices
        for(int i = 0; i < _mesh->get_nb_vertices(); i++)
        {
            Point_cu center = Convs::to_point(_mesh->get_vertex(i));

            std::vector<float> bones_weights(_skel->nb_joints(), 0.f);

            std::map<int, float>& map = weights[i];
            for(unsigned bone_id = 0; bone_id < bones_weights.size(); bone_id++)
            {
                it = map.find(bone_id);
                float vert_weight = 0.f;
                if(it != map.end()) vert_weight = it->second;

                if(vert_weight <= 0.000001f)
                {
                    float min_length = std::numeric_limits<float>::infinity();
                    int dep      = _mesh->get_edge_offset(i*2    );
                    int nb_neigh = _mesh->get_edge_offset(i*2 + 1);
                    for(int n = dep; n < (dep+nb_neigh); n++)
                    {
                        int index_neigh = _mesh->get_edge(n);

                        Point_cu neigh = Convs::to_point(_mesh->get_vertex(index_neigh));
                        float norm = (center-neigh).norm();

                        std::map<int, float>::iterator it_nei;
                        it_nei = weights[index_neigh].find(bone_id);
                        if(it_nei != weights[index_neigh].end())
                        {
                            float w = it_nei->second;
                            if(w > 0.0001f)
                            {
                                if(norm < min_length)
                                {
                                    min_length = norm;
                                    w = w - min_length * strength /** iter*iter*/;
                                    bones_weights[bone_id] = w;
                                }
                            }
                        }
                    }
                }
                else
                    bones_weights[bone_id] = vert_weight;
            }


            for(unsigned j = 0; j < bones_weights.size(); j++)
            {
                float w = bones_weights[j];
                if(w > 0.f){
                    new_weights[i][j] = w;
                }
            }
        }// END nb_vert

        for(int i = 0; i < _mesh->get_nb_vertices(); i++){
            weights[i].swap(new_weights[i]);
            new_weights[i].clear();
        }
    }// END nb_iter

    // Eliminate near zero weigths and normalize :
    for(int i = 0; i < _mesh->get_nb_vertices(); i++)
    {
        std::map<int, float>& map = weights[i];
        float sum = 0.f;
        for(it = map.begin(); it != map.end(); ++it)
        {
            float w = it->second;
            if(w > 0.0001f)
            {
                w = w*w*w;
                new_weights[i][it->first] = w;
                sum += w;
            }
        }

        weights[i].clear();

        std::map<int, float>& n_map = new_weights[i];
        for(it = n_map.begin(); it != n_map.end(); ++it)
        {
            float w = it->second / sum;
            if(w > 0.0001f)
                weights[i][it->first] = w;
            if(w > 1.1f)
            {
                std::cout << w << std::endl;
                assert(false);
            }
        }
    }

    update_device_ssd_weights();
}

// -----------------------------------------------------------------------------

void Animesh::topology_diffuse_ssd_weights(int nb_iter, float strength)
{
    std::vector<std::map<int, float> >& weights = h_weights;

    // diffuse weights
    std::vector<std::map<int, float> > new_weights(weights.size());
    std::map<int, float>::iterator it;
    for(int iter = 0; iter < nb_iter; iter++)
    {
        // For all vertices
        for(int i = 0; i < _mesh->get_nb_vertices(); i++)
        {
            std::vector<float> bones_weights(_skel->nb_joints(), 0);
            // For all vertices' neighborhoods sum the weight of each bone
            int dep      = _mesh->get_edge_offset(i*2    );
            int nb_neigh = _mesh->get_edge_offset(i*2 + 1);
            for(int n = dep; n < (dep+nb_neigh); n++)
            {
                int index_neigh = _mesh->get_edge(n);

                std::map<int, float>& map = weights[index_neigh];
                for(it = map.begin(); it != map.end(); ++it)
                    bones_weights[it->first] += it->second;
            }

            for(unsigned j = 0; j < bones_weights.size(); j++)
            {
                float val = bones_weights[j];
                bones_weights[j] = val * (1.f-strength) + val * strength / (nb_neigh);
            }

            for(unsigned j = 0; j < bones_weights.size(); j++)
            {
                float w = bones_weights[j];
                if(w > 0.f) new_weights[i][j] = w;
            }
        }// END nb_vert

        for(int i = 0; i < _mesh->get_nb_vertices(); i++){
            weights[i].swap(new_weights[i]);
            new_weights[i].clear();
        }
    }// END nb_iter

    // Eliminate near zero weigths and normalize :
    for(int i = 0; i < _mesh->get_nb_vertices(); i++)
    {
        std::map<int, float>& map = weights[i];
        float sum = 0.f;
        for(it = map.begin(); it != map.end(); ++it)
        {
            float w = it->second;
            if(w > 0.0001f)
            {
                new_weights[i][it->first] = w;
                sum += w;
            }
        }

        weights[i].clear();

        std::map<int, float>& n_map = new_weights[i];
        for(it = n_map.begin(); it != n_map.end(); ++it)
        {
            float w = it->second / sum;
            if(w > 0.0001f)
                weights[i][it->first] = w;
        }
    }

    update_device_ssd_weights();
}

// -----------------------------------------------------------------------------

void Animesh::init_smooth_factors(Cuda_utils::DA_float& d_smooth_factors)
{
    const int nb_vert = _mesh->get_nb_vertices();
    HA_float smooth_factors(nb_vert);

    for(int i=0; i<nb_vert; i++)
        smooth_factors[i] = 0.0f;

#if 0
    // We compute at each joint the mean length of every bone connected to that
    // joint, and store it in threshold array multiplied by a scale factor;
    HA_float threshold(_skel->nb_joints());
    const float scale = 0.2f;

    for(int i = 0; i < _skel->nb_joints(); i++){
        if(i == _skel->root()) continue;

        float mean_length = _skel->get_bone(i)->_length;
        const std::vector<int>& sons = _skel->get_sons(i);
        for(unsigned p = 0; p < sons.size(); ++p)
            mean_length += _skel->get_bone(sons[p])->_length;

        mean_length /= (sons.size() + 1);
        threshold[i] = mean_length * scale;
    }


    // Every vertices which projected distance on its nearest bone is below
    // the threshold is smoothen
    for(int i=0; i<nb_vert; i++)
    {
        const int nearest_joint = h_vertices_nearest_joint[i];

        if(!_skel->is_leaf(nearest_joint))
        {
            Point_cu vert = Convs::to_point(_mesh->get_vertex(i));
            const int nearest_bone = h_vertices_nearest_bones[i];

            float dist;

            const Bone* b = _skel->get_bone(nearest_bone);
            if(nearest_bone == nearest_joint)
                dist = b->dist_proj_to(vert);
            else{
                const Vec3_cu op = vert - (b->_org + b->_dir);
                dist = op.dot(-b->_dir.normalized());
            }

            if(dist < threshold[nearest_joint])
                smooth_factors[i] = 1.0f;
        }
    }
#endif
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

void Animesh::get_anim_vertices_aifo(std::vector<float>& anim_vert)
{
    const int nb_vert = d_output_vertices.size();
    anim_vert.reserve(nb_vert);
    Cuda_utils::HA_Point_cu h_out_verts(nb_vert);
    h_out_verts.copy_from(d_output_vertices);

    for(int i = 0; i < nb_vert; i++)
    {
        Point_cu p = h_out_verts[vmap_new_old[i]];
        anim_vert.push_back(p.x);
        anim_vert.push_back(p.y);
        anim_vert.push_back(p.z);
    }
}

// -----------------------------------------------------------------------------

void Animesh::set_bone_type(int id, int bone_type)
{
    _skel->reset();
    Bone* bone = 0;
    const Bone* prev_bone = _skel->get_bone( id );
    float rad = prev_bone->radius();
    switch(bone_type){
    case EBone::PRECOMPUTED:
    {
        // We don't precompute an already precomputed primitive
        assert(_skel->bone_type(id) != EBone::PRECOMPUTED );
        // Precompute a SSD bone is useless and should be forbiden
        assert(_skel->bone_type(id) != EBone::SSD         );

        Bone_precomputed* b = new Bone_precomputed( prev_bone->get_obbox() );
        b->get_primitive().fill_grid_with( _skel->get_bone(id) );
        bone = b;
    }break;
    case EBone::HRBF:     bone = new Bone_hrbf(rad);      break;
    case EBone::CYLINDER: bone = new Bone_cylinder();     break;
    case EBone::SSD:      bone = new Bone_ssd();          break;

    default: //unknown bone type !
        assert(false);
        break;

    }


    bone->set_radius(rad);
    _skel->set_bone(id, bone);

    init_ssd_interpolation_weights();
    _skel->unreset();
}

// -----------------------------------------------------------------------------

float Animesh::get_junction_radius(int bone_id){
    assert(bone_id >=0                 );
    assert(bone_id < _skel->nb_joints());
    return h_junction_radius[bone_id];
}

// -----------------------------------------------------------------------------

void Animesh::set_junction_radius(int bone_id, float rad)
{
    assert(bone_id >=0                 );
    assert(bone_id < _skel->nb_joints());
    h_junction_radius[bone_id] = rad;
}

// -----------------------------------------------------------------------------

void Animesh::set_ssd_weight(int id_vertex, int id_joint, float weight)
{
    id_joint = _skel->parent(id_joint);

    assert(id_vertex < (int)d_input_vertices.size());
    // clamp [0, 1]
    weight = fmax(0.f, fmin(weight, 1.f));

    float old_weight = get_ssd_weight(id_vertex, id_joint);
    float delta      = old_weight - weight;

    int start, end;
    d_jpv.fetch(id_vertex*2  , start);
    d_jpv.fetch(id_vertex*2+1, end  );

    delta = delta / (float)(end-1);

    for(int i=start; i<(start+end); i++)
    {
        int current_joint;
        d_joints.fetch(i, current_joint);
        if(current_joint == id_joint)
            d_weights.set(i, weight);
        else
        {
            float w;
            d_weights.fetch(i, w);
            d_weights.set(i, w+delta);
        }
    }
}

// -----------------------------------------------------------------------------

float Animesh::get_ssd_weight(int id_vertex, int id_joint)
{
    assert(id_vertex < d_input_vertices.size());

    int start, end;
    d_jpv.fetch(id_vertex*2  , start);
    d_jpv.fetch(id_vertex*2+1, end  );

    for(int i=start; i<(start+end); i++)
    {
        int current_joint;
        d_joints.fetch(i, current_joint);
        if(current_joint == id_joint)
        {
            float w;
            d_weights.fetch(i, w);
            return w;
        }
    }

    // Joint "id_joint" is not associated to this vertex
    assert(false);
    return 0.f;
}

// -----------------------------------------------------------------------------

void Animesh::get_ssd_weights(std::vector<std::map<int, float> >& weights)
{
    const int nb_vert = d_input_vertices.size();
    weights.clear();
    weights.resize(nb_vert);

    HA_float h_weights(d_weights.size());
    HA_int h_joints(d_joints.size());
    HA_int h_jpv(d_jpv.size());
    h_weights.copy_from(d_weights);
    h_joints.copy_from(d_joints);
    h_jpv.copy_from(d_jpv);

    for( int i = 0; i < nb_vert; i++)
    {
        int start = h_jpv[i*2];
        int end   = start + h_jpv[i*2 + 1];
        weights[i].clear();
        for(int j = start; j < end ; j++){
            weights[i][h_joints[j]] = h_weights[j];
            //std::cout << h_weights[j] << std::endl;
        }
    }
}

// -----------------------------------------------------------------------------

void Animesh::update_host_ssd_weights()
{
    get_ssd_weights(h_weights);
}

// -----------------------------------------------------------------------------

void Animesh::set_ssd_weights(const std::vector<std::map<int, float> >& in_weights)
{
    const int nb_vert = d_input_vertices.size();
    assert( in_weights.size() == (unsigned)nb_vert );

    std::vector<float> weights;
    std::vector<int>   joints;
    HA_int             jpv(nb_vert*2);

    weights.reserve(nb_vert*2);
    joints.reserve(nb_vert*2);

    int acc = 0;
    for( int i = 0; i < nb_vert; i++)
    {
        const std::map<int, float>& map = in_weights[i];
        jpv[i*2    ] = acc;
        jpv[i*2 + 1] = map.size();
        std::map<int, float>::const_iterator it;
        for(it = map.begin(); it != map.end(); ++it)
        {
            joints.push_back(it->first);
            weights.push_back(it->second);
        }
        acc += map.size();
    }

    d_weights.malloc(weights.size());
    d_joints.malloc(joints.size());
    d_jpv.malloc(jpv.size());
    d_weights.copy_from(weights);
    d_joints.copy_from(joints);
    d_jpv.copy_from(jpv);
}

// -----------------------------------------------------------------------------

void Animesh::update_device_ssd_weights()
{
    set_ssd_weights(h_weights);
}

// -----------------------------------------------------------------------------

void Animesh::export_weights(const char* filename)
{
    using namespace std;
    ofstream file(filename, ios_base::out|ios_base::trunc);

    if(!file.is_open()){
        cerr << "Error exporting file " << filename << endl;
        exit(1);
    }

    // Copy to host :
    HA_int   h_jpv(d_jpv.size());
    HA_int   h_joints(d_joints.size());
    HA_float h_weights(d_weights.size());

    h_jpv.copy_from(d_jpv);
    h_joints.copy_from(d_joints);
    h_weights.copy_from(d_weights);

    for(int i = 0; i < d_input_vertices.size(); i++)
    {
        int start, end;
        float sum_weights = 0.f;
        // vertices are not necessarily
        start = h_jpv[vmap_new_old[i]*2    ];
        end   = h_jpv[vmap_new_old[i]*2 + 1];

        for(int j=start; j<(start+end); j++)
        {
            float weight = h_weights[j];
            int   bone   = h_joints[j];
            sum_weights += weight;

            file << bone << " " << weight << " ";
        }

        if((sum_weights > 1.0001f) || (sum_weights < -0.0001f)){
            std::cerr << "WARNING: exported ssd weights does not sum to one ";
            std::cerr << "(line " << (i+1) << ")" << std::endl;
        }
        file << endl;
    }
    file.close();
}


// -----------------------------------------------------------------------------

void Animesh::read_weights_from_file(const char* filename,
                                          bool file_has_commas)
{
    using namespace std;
    using namespace Cuda_utils;

    ifstream file(filename);

    int n = _mesh -> get_nb_vertices();
    std::vector<float> h_weights; h_weights.reserve(2*n);
    std::vector<int>   h_joints; h_joints.reserve(2*n);
    Host::Array<int>   h_jpv(2*n);

    if(!file.is_open()){
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }

    int k = 0;
    for(int i = 0; i < n; i++)
    {
        std::string str_line;
        std::getline(file, str_line);
        std::stringbuf current_line_sb(str_line, ios_base::in);

        istream current_line(&current_line_sb);
        int j = 0;
        //int j_old = -1;
        float weight, sum_weights = 0.f;
        int p = 0;
        while(!current_line.eof() && !str_line.empty())
        {
            current_line >> j;
            //if(j == j_old) break;
            if(file_has_commas) current_line.ignore(1,',');

            current_line >> weight;

            if(file_has_commas) current_line.ignore(1,',');

            current_line.ignore(10,' ');

            if(current_line.peek() == '\r') current_line.ignore(1,'\r');

            if(current_line.peek() == '\n') current_line.ignore(1,'\n');

            p++;
            h_weights.push_back( weight ); // SSD weight
            h_joints. push_back(  j     ); // joint number
            //j_old = j;

            sum_weights += weight;
            if(j < 0 || j > _skel->nb_joints()){
                std::cerr << "ERROR: incorrect joint id in imported ssd weights.";
                std::cerr << "Maybe the file does not match the skeleton";
                std::cerr << std::endl;
            }
        }

        if((sum_weights > 1.0001f) || (sum_weights < -0.0001f)){
            std::cerr << "WARNING: imported ssd weights does not sum to one ";
            std::cerr << "(line " << (i+1) << ")" << std::endl;
        }

        // we use vmap_old_new because Animesh does not necessarily
        // stores vertices in the same order as in the off file
        h_jpv[2*vmap_old_new[i]  ] = k; //read start position
        h_jpv[2*vmap_old_new[i]+1] = p; //number of joints modifying that vertex
        k += p;
    } // END FOR NB lINES

    // Copy weights to device mem
    d_jpv.copy_from(h_jpv);
    d_weights.malloc(k);
    d_weights.copy_from(h_weights);
    d_joints.malloc(k);
    d_joints.copy_from(h_joints);

    update_host_ssd_weights();

    cout << "file \"" << filename << "\" loaded successfully" << endl;
    set_default_bones_radius();
    file.close();
}

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

void Animesh::update_opengl_buffers(int nb_vert,
                                    const Vec3_cu* vert,
                                    const Vec3_cu* normals,
                                    const Vec3_cu* tangents,
                                    GlBuffer_obj* vbo,
                                    GlBuffer_obj* nbo,
                                    GlBuffer_obj* tbo)
{
    const int block_s = 16;
    const int grid_s  = (nb_vert + block_s - 1) / block_s;

    Vec3_cu* cuda_vbo = 0;
    Vec3_cu* cuda_nbo = 0;
    Vec3_cu* cuda_tbo = 0;
    vbo->cuda_map_to(cuda_vbo);
    nbo->cuda_map_to(cuda_nbo);
    if(tangents != 0 && tbo != 0)
        tbo->cuda_map_to(cuda_tbo);

    // Unpacking vertices and normals for the rendering of the mesh
    // (because of texture vertices which has multiple texture coordinates
    // mut be duplicated)
    Animesh_kers::unpack_vert_and_normals<<<grid_s, block_s >>>
        (vert, normals, tangents, d_packed_vert_map.ptr(), cuda_vbo, cuda_nbo, cuda_tbo, nb_vert);

    CUDA_CHECK_ERRORS();

    vbo->cuda_unmap();
    nbo->cuda_unmap();
    if(tangents != 0 && tbo != 0)
        tbo->cuda_unmap();
}

// -----------------------------------------------------------------------------

void Animesh::reset_flip_propagation(){
    int nb_vert = _mesh->get_nb_vertices();
    Host::Array<bool> flip_prop(nb_vert);
    for(int i = 0; i < nb_vert; i++){
        flip_prop[i] = false;
    }

    d_flip_propagation.copy_from(flip_prop);
}

// -----------------------------------------------------------------------------

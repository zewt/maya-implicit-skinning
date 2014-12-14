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

/**
 * @file animesh_projection.cu
 * @brief implemention of the Animesh class related to deformation
 *
 */

#include "conversions.hpp"
#include "animesh_kers.hpp"
#include "cuda_ctrl.hpp"
#include "timer.hpp"
#include "cuda_current_device.hpp"
#include "std_utils.hpp"

// -----------------------------------------------------------------------------

void Animesh::update_base_potential()
{
    if(!do_update_potential) return;

    Timer time;
    time.start();
    const int nb_verts = d_input_vertices.size();
    const int block_size = 256;
    const int grid_size =
            (nb_verts + block_size - 1) / block_size;

    _skel->reset();

    Animesh_kers::compute_base_potential<<<grid_size, block_size>>>
        (d_input_vertices.ptr(), nb_verts, d_base_potential.ptr(), d_base_gradient.ptr());

    CUDA_CHECK_ERRORS();

    _skel->unreset();

    if(mesh_color == EAnimesh::BASE_POTENTIAL)
        set_colors(mesh_color);

    std::cout << "Update base potential in " << time.stop() << " sec" << std::endl;
}

// -----------------------------------------------------------------------------

void Animesh::update_bone_samples(Bone::Id bone_id,
                                  const std::vector<Vec3_cu>& nodes,
                                  const std::vector<Vec3_cu>& n_nodes)
{
    const float rad_hrbf = _skel->get_hrbf_radius( bone_id );

    // We update nodes in bones
    Bone_hrbf*  hrbf_bone = new Bone_hrbf(rad_hrbf);
    HermiteRBF& hrbf      = hrbf_bone->get_hrbf();

    std::cout << "Solving for " << nodes.size() << " nodes" << std::endl;

    // Solve/compute compute HRBF weights
    Timer t;
    t.start();
    hrbf.init_coeffs(nodes, n_nodes);
    std::cout << "Solving in: " << t.stop() << " sec \n" << std::endl;

    hrbf_bone->set_radius( _skel->get_bone( bone_id)->radius() );
    _skel->set_bone(bone_id, hrbf_bone);

    //update_base_potential();
}

// -----------------------------------------------------------------------------

void Animesh::compute_tangents(const Vec3_cu* vertices, Vec3_cu* tangents)
{
    // TODO: optimize the tangent computation by precomputed coefficients
    // only related top connexity and uv coords.
    int* map_tri   = 0;
    int* map_quad  = 0;
    float* map_tex = 0;
    if(_mesh->_nb_tri  > 0) _mesh->_index_bo_tri. cuda_map_to(map_tri);
    if(_mesh->_nb_quad > 0) _mesh->_index_bo_quad.cuda_map_to(map_quad);
    _mesh->_tex_bo.cuda_map_to(map_tex);

    Animesh_kers::compute_tangents(d_input_tri.ptr(),
                                   d_input_quad.ptr(),
                                   map_tri,
                                   map_quad,
                                   d_piv,
                                   _mesh->get_nb_tri(),
                                   _mesh->get_nb_quad(),
                                   vertices,
                                   map_tex,
                                   d_unpacked_tangents,
                                   _mesh->_max_faces_per_vertex,
                                   tangents);

    if(_mesh->_nb_tri  > 0) _mesh->_index_bo_tri. cuda_unmap();
    if(_mesh->_nb_quad > 0) _mesh->_index_bo_quad.cuda_unmap();
    _mesh->_tex_bo.cuda_unmap();
    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void Animesh::compute_normals(const Vec3_cu* vertices, Vec3_cu* normals)
{
    if(_mesh->get_nb_faces() > 0)
    {
        Animesh_kers::compute_normals(d_input_tri.ptr(),
                                      d_input_quad.ptr(),
                                      d_piv,
                                      _mesh->get_nb_tri(),
                                      _mesh->get_nb_quad(),
                                      vertices,
                                      d_unpacked_normals,
                                      _mesh->_max_faces_per_vertex,
                                      normals);
    }
    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void Animesh::tangential_smooth(float* factors,
                                Vec3_cu* d_vertices,
                                Vec3_cu* d_vertices_prealloc,
                                Vec3_cu* d_normals,
                                int nb_iter)
{
    const int block_size = 256;
    // nb_threads == nb_mesh_vertices
    const int nb_threads = d_edge_list_offsets.size() / 2;
    const int grid_size = (nb_threads + block_size - 1) / block_size;
    Vec3_cu* d_vertices_a = d_vertices;
    Vec3_cu* d_vertices_b = d_vertices_prealloc;
    Vec3_cu* normals   = (Vec3_cu*)d_normals;

    compute_normals(d_vertices, d_normals);

    for(int i = 0; i < nb_iter; i++)
    {
        Animesh_kers::tangential_smooth_kernel_first_pass
                <<<grid_size, block_size>>>(d_vertices_a,  // in vertices
                                            normals,       // in normals
                                            d_vertices_b,  // out vector
                                            d_edge_list.ptr(),
                                            d_edge_list_offsets.ptr(),
                                            factors,
                                            do_local_smoothing,
                                            smooth_force_a,
                                            3,
                                            nb_threads);

        Animesh_kers::tangential_smooth_kernel_final_pass
                <<<grid_size, block_size>>>(d_vertices_a, // in vertices
                                            d_vertices_b, // in vector
                                            d_vertices_b, // res = vert + vec
                                            nb_threads);

        compute_normals(d_vertices, d_normals);
        Utils::swap_pointers(d_vertices_a, d_vertices_b);
    }

    if(nb_iter % 2 == 1)
    {
        Animesh_kers::copy_arrays<<<grid_size, block_size>>>
            (d_vertices_prealloc, d_vertices, nb_threads);
    }
}

// -----------------------------------------------------------------------------

void Animesh::smooth_mesh(Vec3_cu* output_vertices,
                          Vec3_cu* output_normals,
                          float* factors,
                          int nb_iter,
                          bool local_smoothing = true)
{
    if(nb_iter == 0) return;

    Vec3_cu* buff  = (Vec3_cu*)d_vert_buffer.ptr();
    Vec3_cu* buff2 = (Vec3_cu*)d_vert_buffer_2.ptr();

    switch(mesh_smoothing)
    {
    case EAnimesh::LAPLACIAN:
        Animesh_kers::laplacian_smooth(output_vertices, output_normals, d_edge_list,
                                       d_edge_list_offsets, factors, local_smoothing,
                                       smooth_force_a, nb_iter, 3);
        break;
    case EAnimesh::CONSERVATIVE:
        Animesh_kers::conservative_smooth(output_vertices,
                                          buff,
                                          (Vec3_cu*)d_gradient.ptr(),
                                          d_edge_list,
                                          d_edge_list_offsets,
                                          d_edge_mvc,
                                          d_vert_to_fit_base.ptr(),
                                          d_vert_to_fit_base.size(),
                                          true,
                                          smooth_force_a,
                                          nb_iter,
                                          factors,//smooth fac
                                          local_smoothing);// use smooth fac ?
        break;
    case EAnimesh::TANGENTIAL:
        tangential_smooth(factors, output_vertices, buff, output_normals, nb_iter);
        break;
    case EAnimesh::HUMPHREY:

        const int nb_vert    = d_input_vertices.size();
        const int block_size = 16;
        const int grid_size  = (nb_vert + block_size - 1) / block_size;

        Animesh_kers::copy_arrays<<<grid_size, block_size >>>
                 (output_vertices,
                  (Vec3_cu*)d_vert_buffer.ptr(),
                  nb_vert);

        Animesh_kers::hc_laplacian_smooth(d_vert_buffer,
                                          output_vertices,
                                          output_normals,
                                          buff2,
                                          d_edge_list,
                                          d_edge_list_offsets,
                                          factors,
                                          local_smoothing,
                                          smooth_force_a,
                                          smooth_force_b,
                                          nb_iter,
                                          3);
        break;
    }
    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void Animesh::conservative_smooth(Vec3_cu* output_vertices,
                                  Vec3_cu* buff,
                                  const Cuda_utils::DA_int& d_vert_to_fit,
                                  int nb_vert_to_fit,
                                  int nb_iter,
                                  bool use_vert_to_fit)
{
    Animesh_kers::conservative_smooth(output_vertices,
                                      buff,
                                      (Vec3_cu*)d_gradient.ptr(),
                                      d_edge_list,
                                      d_edge_list_offsets,
                                      d_edge_mvc,
                                      d_vert_to_fit.ptr(),
                                      nb_vert_to_fit,
                                      use_vert_to_fit,
                                      smooth_force_a,
                                      nb_iter,
                                      d_smooth_factors_conservative.ptr(),
                                      true);
}

// -----------------------------------------------------------------------------

void Animesh::fit_mesh(int nb_vert_to_fit,
                       int* d_vert_to_fit,
                       bool full_eval,
                       bool smooth_fac_from_iso,
                       Vec3_cu* d_vertices,
                       int nb_steps,
                       float smooth_strength)
{
    if(nb_vert_to_fit == 0) return;

    const int nb_vert    = nb_vert_to_fit;
    const int block_size = 16;
    const int grid_size  = (nb_vert + block_size - 1) / block_size;

    CUDA_CHECK_ERRORS();
    CUDA_CHECK_KERNEL_SIZE(block_size, grid_size);

    Animesh_kers::match_base_potential
        <<<grid_size, block_size >>>
        (full_eval,
         smooth_fac_from_iso,
         d_vertices,
         d_base_potential.ptr(),
         d_ssd_normals.ptr(),
         d_gradient.ptr(),
         d_nearest_bone_in_device_mem.ptr(),
         d_smooth_factors_conservative.ptr(),
         d_smooth_factors_laplacian.ptr(),
         d_vert_to_fit,
         nb_vert_to_fit,
         /* (do_tune_direction && !full_eval), */
         (unsigned short)nb_steps,
         Cuda_ctrl::_debug._collision_threshold,
         Cuda_ctrl::_debug._step_length,
         Cuda_ctrl::_debug._potential_pit,
         (int*)d_vertices_state.ptr(),
         smooth_strength,
         Cuda_ctrl::_debug._collision_depth,
         Cuda_ctrl::_debug._slope_smooth_weight,
         Cuda_ctrl::_debug._raphson);

    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void Animesh::geometric_deformation(EAnimesh::Blending_type t,
                                    const Cuda_utils::DA_Point_cu& d_in,
                                    Vec3_cu* out,
                                    Vec3_cu* out2)
{
    const int block_size = 16;
    const int grid_size  = (d_in.size() + block_size - 1) / block_size;

    if(t == EAnimesh::DUAL_QUAT_BLENDING)
    {
        Animesh_kers::transform_dual_quat<<<grid_size, block_size >>>
            (d_in.ptr(),
             d_base_gradient.ptr(),
             d_in.size(),
             out,
             out2,
             d_ssd_normals.ptr(),
             _skel->d_dual_quat(),
             d_weights.ptr(),
             d_joints.ptr(),
             d_jpv.ptr());
    }
    else /*if(type == MATRIX_BLENDING */
    {
        Animesh_kers::transform_SSD<<<grid_size, block_size >>>
            (d_in.ptr(),
             d_base_gradient.ptr(),
             d_in.size(),
             out,
             out2,
             d_ssd_normals.ptr(),
             _skel->d_transfos(),
             d_weights.ptr(),
             d_joints.ptr(),
             d_jpv.ptr());
    }
    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void Animesh::ssd_lerp(Vec3_cu* out_verts)
{
    const int nb_vert_to_fit = d_vert_to_fit_base.size();
    if( nb_vert_to_fit == 0) return;
    const int block_size = 16;
    const int grid_size  = (nb_vert_to_fit + block_size - 1) / block_size;

    Animesh_kers::lerp_kernel<<<grid_size, block_size>>>
            (d_vert_to_fit_base.ptr(),
             out_verts,
             (Vec3_cu*)d_ssd_vertices.ptr(),
             d_ssd_interpolation_factor.ptr(),
             out_verts,
             nb_vert_to_fit);

    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void Animesh::transform_vertices(EAnimesh::Blending_type type)
{
    using namespace Cuda_ctrl;
    //Timer time;
    //time.start();

    const int nb_vert    = d_input_vertices.size();

    Vec3_cu* ssd_verts    = (Vec3_cu*)d_ssd_vertices.ptr();
    Vec3_cu* out_verts    = (Vec3_cu*)d_output_vertices.ptr();
    Vec3_cu* out_normals  = (Vec3_cu*)d_output_normals.ptr();
    Vec3_cu* out_tangents = (Vec3_cu*)d_output_tangents.ptr();
//    Vec3_cu* ssd_normals  = (Vec3_cu*)d_ssd_normals.ptr();

#if 0
    d_output_vertices.copy_from(d_input_vertices);
#else
    geometric_deformation(type, d_input_vertices, out_verts, ssd_verts);
#endif

    if(do_implicit_skinning)
    {
        d_smooth_factors_laplacian.copy_from( d_input_smooth_factors );
        d_vert_to_fit.copy_from(d_vert_to_fit_base);
        int nb_vert_to_fit = d_vert_to_fit.size();
        const int nb_steps = Cuda_ctrl::_debug._nb_step;

        Cuda_utils::DA_int* curr = &d_vert_to_fit;
#if 1

        if(do_smooth_mesh)
        {
            Cuda_utils::DA_int* prev = &d_vert_to_fit_buff;
            // Interleaved fittig
            for( int i = 0; i < nb_steps && nb_vert_to_fit != 0; i++)
            {
                // First fitting (only evaluate the three adjacent nearest bones)
                if(nb_vert_to_fit > 0){
                    fit_mesh(nb_vert_to_fit, curr->ptr(), false/*full fit*/, true/*smooth from iso*/, out_verts, 2, smooth_force_a);
                }

                nb_vert_to_fit = pack_vert_to_fit_gpu(*curr, d_vert_to_fit_buff_scan, *prev, nb_vert_to_fit );
                Utils::swap_pointers(curr, prev);

                // user smoothing
                //smooth_mesh(output_vertices, output_normals, d_smooth_factors.ptr(), smoothing_iter, false/*local smoothing*/);
                conservative_smooth(out_verts, out_normals, *curr, nb_vert_to_fit, smoothing_iter);
            }
        }
        else
        {
            // First fitting
            if(nb_vert_to_fit > 0)
            {
                d_vert_to_fit.copy_from(d_vert_to_fit_base);
                //compute_normals(output_vertices, ssd_normals);
                fit_mesh(nb_vert_to_fit, curr->ptr(), true/*full fit*/, false/*smooth from iso*/, out_verts, nb_steps, _debug._smooth1_force);
            }
        }

#else
        // First fitting
        if(nb_vert_to_fit > 0){
            d_vert_to_fit.copy_from(d_vert_to_fit_base);
            //compute_normals(output_vertices, ssd_normals);
            fit_mesh(nb_vert_to_fit, curr->ptr(), true/*full fit*/, false/*smooth from iso*/, out_verts, nb_steps, _debug._smooth1_force);
        }
#endif

#if 1
        // Smooth the initial guess
        if(_debug._smooth_mesh)
        {
            this->diffuse_attr(diffuse_smooth_weights_iter, 1.f, d_smooth_factors_laplacian.ptr());
            smooth_mesh(out_verts, out_normals, d_smooth_factors_laplacian.ptr(), _debug._smooth1_iter);
        }

        // Final fitting (global evaluation of the skeleton)
        if(Cuda_ctrl::_debug._fit_on_all_bones)
        {
            //compute_normals(output_vertices, ssd_normals);
            curr->copy_from(d_vert_to_fit_base);
            fit_mesh(curr->size(), curr->ptr(), true/*full fit*/, false/*smooth from iso*/, out_verts, nb_steps, _debug._smooth2_force);
        }

        // Final smoothing
        if(_debug._smooth_mesh)
        {
            this->diffuse_attr(diffuse_smooth_weights_iter, 1.f, d_smooth_factors_laplacian.ptr());
            smooth_mesh(out_verts, out_normals, d_smooth_factors_laplacian.ptr(), _debug._smooth2_iter);
        }
#endif

        // Interpolation between ssd position and correction:
        ssd_lerp(out_verts);
    }

    compute_normals(out_verts, out_normals);

    if(_mesh->_has_tex_coords && _mesh->_has_bumpmap)
        compute_tangents(out_verts, out_normals);
    else
        out_tangents = 0;

    // Fill the buffer objects with the deformed vertices.
    // vertex with multiple texture coordinates are duplicated
    update_opengl_buffers(nb_vert,
                          out_verts,
                          out_normals,
                          out_tangents,
                          &(_mesh->_vbo),
                          &(_mesh->_normals_bo),
                          &(_mesh->_tangents_bo));

#if !defined(NDEBUG) || 1
    if(mesh_color == EAnimesh::ANIM_SMOOTH_CONSERVATIVE ||
       mesh_color == EAnimesh::ANIM_SMOOTH_LAPLACIAN ||
       mesh_color == EAnimesh::NORMAL ||
       mesh_color == EAnimesh::GRAD_POTENTIAL ||
       mesh_color == EAnimesh::VERTICES_STATE)
        set_colors(mesh_color);
#endif

    //std::cout << "transform :" << (float)time.stop()/1000.f << std::endl;$
}

// -----------------------------------------------------------------------------

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

#include "animesh_kers.hpp"
#include "cuda_ctrl.hpp"
#include "timer.hpp"
#include "cuda_current_device.hpp"
#include "std_utils.hpp"

void Animesh::update_base_potential()
{
    Timer time;
    time.start();
    const int nb_verts = d_input_vertices.size();
    const int block_size = 256;
    const int grid_size =
            (nb_verts + block_size - 1) / block_size;

    assert(d_input_vertices.ptr());
    assert(d_base_potential.ptr());

    Animesh_kers::compute_base_potential<<<grid_size, block_size>>>
        (_skel->get_skel_id(), d_input_vertices.ptr(), nb_verts, d_base_potential.ptr());

    CUDA_CHECK_ERRORS();

    std::cout << "Update base potential in " << time.stop() << " sec" << std::endl;
}

void Animesh::get_base_potential(std::vector<float> &pot) const
{
    pot = d_base_potential.to_host_vector();
}

void Animesh::set_base_potential(const std::vector<float> &pot)
{
    d_base_potential.malloc(get_nb_vertices());
    d_base_potential.copy_from(pot);
}

void Animesh::compute_normals(const Vec3_cu* vertices, Vec3_cu* normals)
{
    if(_mesh->get_nb_faces() == 0)
        return;

    Animesh_kers::compute_normals(d_input_tri.ptr(),
                                    d_piv,
                                    _mesh->get_nb_tri(),
                                    vertices,
                                    d_unpacked_normals,
                                    _mesh->_max_faces_per_vertex,
                                    normals);
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
    Vec3_cu* normals   = d_normals;

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
                          Vec3_cu* output_vertices_temp,
                          float* factors,
                          int nb_iter,
                          bool local_smoothing = true)
{
    if(nb_iter == 0) return;

    Vec3_cu* buff  = d_vert_buffer.ptr();
    Vec3_cu* buff2 = d_vert_buffer_2.ptr();

    switch(mesh_smoothing)
    {
    case EAnimesh::NONE:
        break;
    case EAnimesh::LAPLACIAN:
        Animesh_kers::laplacian_smooth(output_vertices, output_vertices_temp, d_edge_list,
                                       d_edge_list_offsets, factors, local_smoothing,
                                       smooth_force_a, nb_iter, 3);
        break;
    case EAnimesh::CONSERVATIVE:
        Animesh_kers::conservative_smooth(output_vertices,
                                          buff,
                                          d_gradient.ptr(),
                                          d_edge_list,
                                          d_edge_list_offsets,
                                          d_edge_mvc,
                                          d_vert_to_fit_base.ptr(),
                                          d_vert_to_fit_base.size(),
                                          smooth_force_a,
                                          nb_iter,
                                          factors,//smooth fac
                                          local_smoothing);// use smooth fac ?
        break;
    case EAnimesh::TANGENTIAL:
        tangential_smooth(factors, output_vertices, buff, output_vertices_temp, nb_iter);
        break;
    case EAnimesh::HUMPHREY:

        const int nb_vert    = d_input_vertices.size();
        const int block_size = 16;
        const int grid_size  = (nb_vert + block_size - 1) / block_size;

        Animesh_kers::copy_arrays<<<grid_size, block_size >>>(output_vertices, d_vert_buffer.ptr(), nb_vert);

        Animesh_kers::hc_laplacian_smooth(d_vert_buffer,
                                          output_vertices,
                                          output_vertices_temp,
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
                                  int nb_iter)
{
    Animesh_kers::conservative_smooth(output_vertices,
                                      buff,
                                      d_gradient.ptr(),
                                      d_edge_list,
                                      d_edge_list_offsets,
                                      d_edge_mvc,
                                      d_vert_to_fit.ptr(),
                                      nb_vert_to_fit,
                                      smooth_force_a,
                                      nb_iter,
                                      d_smooth_factors_conservative.ptr(),
                                      true);
}

// -----------------------------------------------------------------------------

void Animesh::fit_mesh(int nb_vert_to_fit,
                       int* d_vert_to_fit,
                       bool smooth_fac_from_iso,
                       Vec3_cu* d_vertices,
                       int nb_steps,
                       float smooth_strength)
{
    if(nb_vert_to_fit == 0) return;

    assert(d_base_potential.ptr());
    assert(d_smooth_factors_conservative.ptr());
    assert(d_smooth_factors_laplacian.ptr());
    assert(d_vertices_state.ptr());

    const int nb_vert    = nb_vert_to_fit;
    const int block_size = 16;
    const int grid_size  = (nb_vert + block_size - 1) / block_size;

    CUDA_CHECK_ERRORS();
    CUDA_CHECK_KERNEL_SIZE(block_size, grid_size);

    Animesh_kers::match_base_potential
        <<<grid_size, block_size >>>
        (_skel->get_skel_id(),
         smooth_fac_from_iso,
         d_vertices,
         d_base_potential.ptr(),
         d_gradient.ptr(),
         d_smooth_factors_conservative.ptr(),
         d_smooth_factors_laplacian.ptr(),
         d_vert_to_fit,
         nb_vert_to_fit,
         /* (do_tune_direction && !full_eval), */
         (unsigned short)nb_steps,
         Cuda_ctrl::_debug._collision_threshold,
         Cuda_ctrl::_debug._step_length,
         Cuda_ctrl::_debug._potential_pit,
         d_vertices_state.ptr(),
         smooth_strength,
         Cuda_ctrl::_debug._collision_depth,
         Cuda_ctrl::_debug._slope_smooth_weight,
         Cuda_ctrl::_debug._raphson);

    CUDA_CHECK_ERRORS();
}

void Animesh::transform_vertices()
{
    const int nb_vert    = d_input_vertices.size();

    // XXX: This is actually Point_cu; we should probably adjust the calls below to allow using
    // that type, instead of casting Vec3_cu to Point_cu.
    Vec3_cu* out_verts    = (Vec3_cu*)d_output_vertices.ptr();
    d_output_vertices.copy_from(d_input_vertices);

    d_smooth_factors_laplacian.copy_from( d_input_smooth_factors );
    // d_vert_to_fit_base: a list of vertices that fit_mesh should be applied to;
    // doesn't depend on the results of skinning
    d_vert_to_fit.copy_from(d_vert_to_fit_base);
    int nb_vert_to_fit = d_vert_to_fit.size();
    const int nb_steps = nb_transform_steps;

    Cuda_utils::DA_int* curr = &d_vert_to_fit;

    if(do_smooth_mesh)
    {
        Cuda_utils::DA_int* prev = &d_vert_to_fit_buff;
        // Interleaved fitting
        // Should we be doing nb_steps/2 steps here, since we're doing two steps per iteration?
        for( int i = 0; i < nb_steps && nb_vert_to_fit != 0; i++)
        {
            fit_mesh(nb_vert_to_fit, curr->ptr(), true/*smooth from iso*/, out_verts, 2, smooth_force_a);

            nb_vert_to_fit = pack_vert_to_fit_gpu(*curr, d_vert_to_fit_buff_scan, *prev, nb_vert_to_fit );
            Utils::swap_pointers(curr, prev);

            // user smoothing
            //smooth_mesh(output_vertices, output_normals, d_smooth_factors.ptr(), smoothing_iter, false/*local smoothing*/);
            conservative_smooth(out_verts, d_vert_buffer.ptr(), *curr, nb_vert_to_fit, smoothing_iter);
        }
    }
    else
    {
        // First fitting
        if(nb_vert_to_fit > 0)
        {
            d_vert_to_fit.copy_from(d_vert_to_fit_base);
            fit_mesh(nb_vert_to_fit, curr->ptr(), false/*smooth from iso*/, out_verts, nb_steps, Cuda_ctrl::_debug._smooth1_force);
        }
    }

#if 1
    // Smooth the initial guess
    if(Cuda_ctrl::_debug._smooth_mesh)
    {
        this->diffuse_attr(diffuse_smooth_weights_iter, 1.f, d_smooth_factors_laplacian.ptr());
        smooth_mesh(out_verts, d_vert_buffer.ptr(), d_smooth_factors_laplacian.ptr(), Cuda_ctrl::_debug._smooth1_iter);
    }

    // Final fitting (global evaluation of the skeleton)
    if(final_fitting)
    {
        // Reset d_vert_to_fit, so we always re-fit all vertices on this pass.
        curr->copy_from(d_vert_to_fit_base);
        fit_mesh(curr->size(), curr->ptr(), false/*smooth from iso*/, out_verts, nb_steps, Cuda_ctrl::_debug._smooth2_force);
    }

    // Final smoothing
    if(Cuda_ctrl::_debug._smooth_mesh)
    {
        this->diffuse_attr(diffuse_smooth_weights_iter, 1.f, d_smooth_factors_laplacian.ptr());
        smooth_mesh(out_verts, d_vert_buffer.ptr(), d_smooth_factors_laplacian.ptr(), Cuda_ctrl::_debug._smooth2_iter);
    }
#endif
}

// -----------------------------------------------------------------------------

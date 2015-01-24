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
#include "animesh_kers.hpp"

#include "conversions.hpp"
#include "cuda_current_device.hpp"
#include "std_utils.hpp"
#include "skeleton_env_evaluator.hpp"
#include "animesh_enum.hpp"
#include "cuda_utils.hpp"
#include "ray_cu.hpp"
#include "bone.hpp"

#include <math_constants.h>

// Max number of binary search steps
#define BINARY_SEARCH_STEPS (20)
#define EPSILON 0.0001f
#define ENABLE_COLOR
#define ENABLE_MARCH

#ifndef PI
#define PI (3.14159265358979323846f)
#endif

// =============================================================================
namespace Animesh_kers{
// =============================================================================

__device__
Vec3_cu compute_rotation_axis(const Transfo& t)
{
    Mat3_cu m = t.get_mat3().get_ortho();
    Vec3_cu axis;
    float angle = m.get_rotation_axis_angle(axis);
    return axis;
}

/// Clean temporary storage
__global__
void clean_unpacked_normals(Device::Array<Vec3_cu> unpacked_normals)
{
    int n = unpacked_normals.size();
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if( p < n){
        unpacked_normals[p] = Vec3_cu(0.f, 0.f, 0.f);
    }
}

// -----------------------------------------------------------------------------

/// Compute the normal of triangle pi
__device__ Vec3_cu
compute_normal_tri(const Mesh::PrimIdx& pi, const Vec3_cu* prim_vertices) {
    const Point_cu va = Convs::to_point(prim_vertices[pi.a]);
    const Point_cu vb = Convs::to_point(prim_vertices[pi.b]);
    const Point_cu vc = Convs::to_point(prim_vertices[pi.c]);
    return ((vb - va).cross(vc - va)).normalized();
}

// -----------------------------------------------------------------------------

/** Assign the normal of each face to each of its vertices
  */
__global__ void
compute_unpacked_normals_tri(const int* faces,
                             Device::Array<Mesh::PrimIdxVertices> piv,
                             int nb_faces,
                             const Vec3_cu* vertices,
                             Device::Array<Vec3_cu> unpacked_normals,
                             int unpack_factor){
    int n = nb_faces;
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p >= n)
        return;

    Mesh::PrimIdx pidx;
    pidx.a = faces[3*p    ];
    pidx.b = faces[3*p + 1];
    pidx.c = faces[3*p + 2];
    Mesh::PrimIdxVertices pivp = piv[p];
    Vec3_cu nm = compute_normal_tri(pidx, vertices);
    int ia = pidx.a * unpack_factor + pivp.ia;
    int ib = pidx.b * unpack_factor + pivp.ib;
    int ic = pidx.c * unpack_factor + pivp.ic;
    unpacked_normals[ia] = nm;
    unpacked_normals[ib] = nm;
    unpacked_normals[ic] = nm;
}

/// Average the normals assigned to each vertex
__global__
void pack_normals( Device::Array<Vec3_cu> unpacked_normals,
                  int unpack_factor,
                  Vec3_cu* normals)
{
    int n = unpacked_normals.size() / unpack_factor;
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        Vec3_cu nm = Vec3_cu::zero();
        for(int i = 0; i < unpack_factor; i++){
            nm = nm + unpacked_normals[p * unpack_factor + i];
        }
        normals[p] = nm.normalized();
    }
}

// -----------------------------------------------------------------------------

/// Compute the normals of the mesh using the normal at each face
void compute_normals(const int* tri,
                     Device::Array<Mesh::PrimIdxVertices> piv,
                     int nb_tri,
                     const Vec3_cu* vertices,
                     Device::Array<Vec3_cu> unpacked_normals,
                     int unpack_factor,
                     Vec3_cu* out_normals)
{

    const int block_size = 512;
    const int nb_threads_clean = unpacked_normals.size();
    const int grid_size_clean = (nb_threads_clean + block_size - 1) / block_size;
    const int nb_threads_pack = unpacked_normals.size() / unpack_factor;
    const int grid_size_pack = (nb_threads_pack + block_size - 1) / block_size;

    const int nb_threads_compute_tri = nb_tri;
    const int grid_size_compute_tri = (nb_threads_compute_tri + block_size - 1) / block_size;

    CUDA_CHECK_KERNEL_SIZE(block_size, grid_size_clean);
    clean_unpacked_normals<<< grid_size_clean, block_size>>>(unpacked_normals);

    CUDA_CHECK_ERRORS();

    if(nb_tri > 0){
#if 1
        compute_unpacked_normals_tri<<< grid_size_compute_tri, block_size>>>
                                   (tri,
                                    piv,
                                    nb_tri,
                                    vertices,
                                    unpacked_normals,unpack_factor);
#else
        compute_unpacked_normals_tri_debug_cpu
                                   (tri,
                                    piv,
                                    nb_tri,
                                    vertices,
                                    unpacked_normals,
                                    unpack_factor,
                                    block_size,
                                    grid_size_compute_tri);
#endif
        CUDA_CHECK_ERRORS();
    }

    pack_normals<<< grid_size_pack, block_size>>>( unpacked_normals,
                                                  unpack_factor,
                                                  out_normals);
    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

/// Compute the tangent of triangle pi
__device__ Vec3_cu
compute_tangent_tri(const Mesh::PrimIdx& pi,
                    const Mesh::PrimIdx& upi,
                    const Vec3_cu* prim_vertices,
                    const float* tex_coords)
{
    const Point_cu va = Convs::to_point(prim_vertices[pi.a]);
    const Point_cu vb = Convs::to_point(prim_vertices[pi.b]);
    const Point_cu vc = Convs::to_point(prim_vertices[pi.c]);

    float2 st1 = { tex_coords[upi.b*2    ] - tex_coords[upi.a*2    ],
                   tex_coords[upi.b*2 + 1] - tex_coords[upi.a*2 + 1]};

    float2 st2 = { tex_coords[upi.c*2    ] - tex_coords[upi.a*2    ],
                   tex_coords[upi.c*2 + 1] - tex_coords[upi.a*2 + 1]};

    const Vec3_cu e1 = vb - va;
    const Vec3_cu e2 = vc - va;

    float coef = 1.f / (st1.x * st2.y - st2.x * st1.y);
    Vec3_cu tangent;
    tangent.x = coef * ((e1.x * st2.y)  + (e2.x * -st1.y));
    tangent.y = coef * ((e1.y * st2.y)  + (e2.y * -st1.y));
    tangent.z = coef * ((e1.z * st2.y)  + (e2.z * -st1.y));

    return tangent;
}

// -----------------------------------------------------------------------------

__global__
void conservative_smooth_kernel(Vec3_cu* in_vertices,
                                Vec3_cu* out_verts,
                                const Vec3_cu* normals,
                                const int* edge_list,
                                const int* edge_list_offsets,
                                const float* edge_mvc,
                                const int* vert_to_fit,
                                bool use_vert_to_fit,
                                float force,
                                int nb_verts,
                                const float* smooth_fac,
                                bool use_smooth_fac)
{
    int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(thread_idx < nb_verts)
    {
        const int p = vert_to_fit[thread_idx];

        const Vec3_cu n       = normals[p].normalized();
        const Vec3_cu in_vert = in_vertices[p];

        if(n.norm() < 0.00001f){
            out_verts[p] = in_vert;
            return;
        }

        Vec3_cu cog(0.f, 0.f, 0.f);

        const int offset = edge_list_offsets[2*p  ];
        const int nb_ngb = edge_list_offsets[2*p+1];

        float sum = 0.f;
        for(int i = offset; i < offset + nb_ngb; i++){
            const int j = edge_list[i];
            const float mvc = edge_mvc[i];
            sum += mvc;
            cog =  cog + in_vertices[j] * mvc;
        }

        if( fabs(sum) < 0.00001f ){
            out_verts[p] = in_vert;
            return;
        }

        cog = cog * (1.f/sum);

        // this force the smoothing to be only tangential :
        const Vec3_cu cog_proj = n.proj_on_plane(Convs::to_point(in_vert), Convs::to_point(cog));
        // this is more like a conservative laplacian smoothing
        //const Vec3_cu cog_proj = cog;

        const float u = use_smooth_fac ? smooth_fac[p] : force;
        out_verts[p]  = cog_proj * u + in_vert * (1.f - u);
    }
}

// -----------------------------------------------------------------------------

template< class T >
__global__ static
void copy_vert_to_fit(const T* d_in,
                      T* d_out,
                      const int* vert_to_fit,
                      bool use_vert_to_fit,
                      int n)
{
    int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(thread_idx < n){
        const int p = vert_to_fit[thread_idx];
        d_out[p] = d_in[p];
    }
}

// -----------------------------------------------------------------------------

void conservative_smooth(Vec3_cu* d_verts,
                         Vec3_cu* d_buff_verts,
                         Vec3_cu* d_normals,
                         const DA_int& d_edge_list,
                         const DA_int& d_edge_list_offsets,
                         const DA_float& d_edge_mvc,
                         const int* d_vert_to_fit,
                         int nb_vert_to_fit,
                         bool use_vert_to_fit,
                         float strength,
                         int nb_iter,
                         const float* smooth_fac,
                         bool use_smooth_fac)
{
    if(nb_vert_to_fit == 0) return;

    const int block_size = 256;
    // nb_threads == nb_mesh_vertices
    const int nb_threads = nb_vert_to_fit;
    const int grid_size  = (nb_threads + block_size - 1) / block_size;
    Vec3_cu* d_verts_a = d_verts;
    Vec3_cu* d_verts_b = d_buff_verts;

    if(nb_iter > 1){
        // TODO: we could only copy vert_to_fit and there neighbors and avoid
        // copy over every elements.
        Cuda_utils::mem_cpy_dtd(d_buff_verts, d_verts, d_edge_list_offsets.size()/2);
    }

    for(int i = 0; i < nb_iter; i++)
    {
        conservative_smooth_kernel<<<grid_size, block_size>>>(d_verts_a,
                                                              d_verts_b,
                                                              d_normals,
                                                              d_edge_list.ptr(),
                                                              d_edge_list_offsets.ptr(),
                                                              d_edge_mvc.ptr(),
                                                              d_vert_to_fit,
                                                              use_vert_to_fit,
                                                              strength,
                                                              nb_vert_to_fit,
                                                              smooth_fac,
                                                              use_smooth_fac);
        CUDA_CHECK_ERRORS();

        Utils::swap_pointers(d_verts_a, d_verts_b);
    }

    if(nb_iter % 2 == 1){
        // d_vertices[n] = d_tmp_vertices[n]
        copy_vert_to_fit<<<grid_size, block_size>>>
            (d_buff_verts, d_verts, d_vert_to_fit, use_vert_to_fit, nb_threads);
        CUDA_CHECK_ERRORS();
    }
}

// -----------------------------------------------------------------------------

__global__
void laplacian_smooth_kernel(const Vec3_cu* in_vertices,
                             Vec3_cu* output_vertices,
                             const int* edge_list,
                             const int* edge_list_offsets,
                             const float* factors,
                             bool use_smooth_factors,
                             float strength,
                             int nb_min_neighbours,
                             int n)
{
        int p = blockIdx.x * blockDim.x + threadIdx.x;
        if(p < n)
        {
            Vec3_cu in_vertex = in_vertices[p];
            Vec3_cu centroid  = Vec3_cu(0.f, 0.f, 0.f);
            float   factor    = factors[p];

            int offset = edge_list_offsets[2*p  ];
            int nb_ngb = edge_list_offsets[2*p+1];
            if(nb_ngb > nb_min_neighbours)
            {
                for(int i = offset; i < offset + nb_ngb; i++){
                    int j = edge_list[i];
                    centroid += in_vertices[j];
                }

                centroid = centroid * (1.f/nb_ngb);

                if(use_smooth_factors)
                    output_vertices[p] = centroid * factor + in_vertex * (1.f-factor);
                else
                    output_vertices[p] = centroid * strength + in_vertex * (1.f-strength);
            }
            else
                output_vertices[p] = in_vertex;
        }

}

// -----------------------------------------------------------------------------

void laplacian_smooth(Vec3_cu* d_vertices,
                      Vec3_cu* d_tmp_vertices,
                      DA_int d_edge_list,
                      DA_int d_edge_list_offsets,
                      float* factors,
                      bool use_smooth_factors,
                      float strength,
                      int nb_iter,
                      int nb_min_neighbours)
{
    const int block_size = 256;
    // nb_threads == nb_mesh_vertices
    const int nb_threads = d_edge_list_offsets.size() / 2;
    const int grid_size = (nb_threads + block_size - 1) / block_size;
    Vec3_cu* d_vertices_a = d_vertices;
    Vec3_cu* d_vertices_b = d_tmp_vertices;
    for(int i = 0; i < nb_iter; i++)
    {
        laplacian_smooth_kernel<<<grid_size, block_size>>>(d_vertices_a,
                                                           d_vertices_b,
                                                           d_edge_list.ptr(),
                                                           d_edge_list_offsets.ptr(),
                                                           factors,
                                                           use_smooth_factors,
                                                           strength,
                                                           nb_min_neighbours,
                                                           nb_threads);
        CUDA_CHECK_ERRORS();
        Utils::swap_pointers(d_vertices_a, d_vertices_b);
    }

    if(nb_iter % 2 == 1){
        // d_vertices[n] = d_tmp_vertices[n]
        copy_arrays<<<grid_size, block_size>>>(d_tmp_vertices, d_vertices, nb_threads);
        CUDA_CHECK_ERRORS();
    }
}

// -----------------------------------------------------------------------------

__global__
void tangential_smooth_kernel_first_pass(const Vec3_cu* in_vertices,
                                         const Vec3_cu* in_normals,
                                         Vec3_cu* out_vector,
                                         const int* edge_list,
                                         const int* edge_list_offsets,
                                         const float* factors,
                                         bool use_smooth_factors,
                                         float strength,
                                         int nb_min_neighbours,
                                         int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n)
    {
        Vec3_cu in_vertex = in_vertices[p];
        Vec3_cu in_normal = in_normals[p];
        Vec3_cu centroid  = Vec3_cu(0.f, 0.f, 0.f);
        float     factor    = factors[p];

        int offset = edge_list_offsets[2*p  ];
        int nb_ngb = edge_list_offsets[2*p+1];
        if(nb_ngb > nb_min_neighbours)
        {
            for(int i = offset; i < offset + nb_ngb; i++){
                int j = edge_list[i];
                centroid += in_vertices[j];
            }

            centroid = centroid * (1.f/nb_ngb);

            if(use_smooth_factors)
                centroid = centroid * factor + in_vertex * (1.f-factor);
            else
                centroid = centroid * strength + in_vertex * (1.f-strength);

            Vec3_cu u = centroid - in_vertex;

            out_vector[p] = u - (in_normal * u.dot(in_normal));
        }
        else
            out_vector[p] = Vec3_cu(0.f, 0.f, 0.f);
    }

}

// -----------------------------------------------------------------------------

__global__
void tangential_smooth_kernel_final_pass(const Vec3_cu* in_vertices,
                                         const Vec3_cu* in_vector,
                                         Vec3_cu* out_vertices,
                                         int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n)
        out_vertices[p] = in_vertices[p] + in_vector[p];
}

// -----------------------------------------------------------------------------

__global__
void hc_smooth_kernel_first_pass(const Vec3_cu* original_vertices,
                                 const Vec3_cu* in_vertices,
                                 Vec3_cu* out_vector,
                                 const int* edge_list,
                                 const int* edge_list_offsets,
                                 const float* factors,
                                 bool use_smooth_factors,
                                 float alpha,
                                 int nb_min_neighbours,
                                 int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n)
    {
        Vec3_cu in_vertex = in_vertices[p];
        Vec3_cu centroid  = Vec3_cu(0.f, 0.f, 0.f);
        float     factor  = factors[p];

        int offset = edge_list_offsets[2*p  ];
        int nb_ngb = edge_list_offsets[2*p+1];
        if(nb_ngb > nb_min_neighbours)
        {
            for(int i = offset; i < offset + nb_ngb; i++){
                int j = edge_list[i];
                centroid += in_vertices[j];
            }

            centroid = centroid * (1.f/nb_ngb);

            if(use_smooth_factors)
                centroid = centroid * factor + in_vertex * (1.f-factor);

            out_vector[p] = centroid - (original_vertices[p]*alpha + in_vertex*(1.f-alpha));
        }
        else
            out_vector[p] = centroid;
    }

}

// -----------------------------------------------------------------------------

__global__
void hc_smooth_kernel_final_pass(const Vec3_cu* in_vectors,
                                 const Vec3_cu* in_vertices,
                                 Vec3_cu* out_vertices,
                                 float beta,
                                 const int* edge_list,
                                 const int* edge_list_offsets,
                                 int nb_min_neighbours,
                                 int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n)
    {
        Vec3_cu centroid = Vec3_cu(0.f, 0.f, 0.f);
        Vec3_cu mean_vec = Vec3_cu(0.f, 0.f, 0.f);
        Vec3_cu in_vec   = in_vectors[p];

        int offset = edge_list_offsets[2*p  ];
        int nb_ngb = edge_list_offsets[2*p+1];

        if(nb_ngb > nb_min_neighbours)
        {
            for(int i = offset; i < offset + nb_ngb; i++){
                int j = edge_list[i];
                centroid += in_vertices[j];
                mean_vec += in_vectors [j];
            }

            float div = 1.f/nb_ngb;
            centroid = centroid * div;
            mean_vec = mean_vec * div;

            Vec3_cu vec = in_vec*beta + mean_vec*(1.f-beta);
            out_vertices[p] = centroid - vec;
        }
        else
            out_vertices[p] = in_vertices[p];
    }

}

// -----------------------------------------------------------------------------

/// @param d_input_vertices vertices in resting pose

void hc_laplacian_smooth(const DA_Vec3_cu& d_original_vertices,
                         Vec3_cu* d_smoothed_vertices,
                         Vec3_cu* d_vector_correction,
                         Vec3_cu* d_tmp_vertices,
                         DA_int d_edge_list,
                         DA_int d_edge_list_offsets,
                         float* factors,
                         bool use_smooth_factors,
                         float alpha,
                         float beta,
                         int nb_iter,
                         int nb_min_neighbours)
{
    const int block_size = 256;
    // nb_threads == nb_mesh_vertices
    const int nb_threads = d_edge_list_offsets.size() / 2;
    const int grid_size = (nb_threads + block_size - 1) / block_size;
    Vec3_cu* d_vertices_a = d_smoothed_vertices;
    Vec3_cu* d_vertices_b = d_tmp_vertices;

    for(int i = 0; i < nb_iter; i++)
    {
        hc_smooth_kernel_first_pass
                <<<grid_size, block_size>>>(d_original_vertices.ptr(),
                                            d_vertices_a,         // in vert
                                            d_vector_correction,  // out vec
                                            d_edge_list.ptr(),
                                            d_edge_list_offsets.ptr(),
                                            factors,
                                            use_smooth_factors,
                                            alpha,
                                            nb_min_neighbours,
                                            nb_threads);
        CUDA_CHECK_ERRORS();

        hc_smooth_kernel_final_pass
                <<<grid_size, block_size>>>(d_vector_correction,
                                            d_vertices_a,
                                            d_vertices_b,
                                            beta,
                                            d_edge_list.ptr(),
                                            d_edge_list_offsets.ptr(),
                                            nb_min_neighbours,
                                            nb_threads);
        CUDA_CHECK_ERRORS();

        Utils::swap_pointers(d_vertices_a, d_vertices_b);
    }

    if(nb_iter % 2 == 1){
        // d_vertices[n] = d_tmp_vertices[n]
        copy_arrays<<<grid_size, block_size>>>(d_tmp_vertices, d_smoothed_vertices, nb_threads);
        CUDA_CHECK_ERRORS();
    }
}

// -----------------------------------------------------------------------------

__global__
void diffusion_kernel(const float* in_values,
                      float* out_values,
                      const int* edge_list,
                      const int* edge_list_offsets,
                      float strength,
                      int nb_vert)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_vert)
    {
        const float in_val   = in_values[p];
        float centroid = 0.f;

        const int offset = edge_list_offsets[2*p  ];
        const int nb_ngb = edge_list_offsets[2*p+1];

        for(int i = offset; i < (offset + nb_ngb); i++)
        {
            const int j = edge_list[i];
            centroid += in_values[j];
        }

        centroid = centroid * (1.f/nb_ngb);

        out_values[p] = centroid * strength + in_val * (1.f-strength);
    }
}

// -----------------------------------------------------------------------------

void diffuse_values(float* d_values,
                    float* d_values_buffer,
                    DA_int d_edge_list,
                    DA_int d_edge_list_offsets,
                    float strength,
                    int nb_iter)
{

    const int block_size = 256;
    // nb_threads == nb_mesh_vertices
    const int nb_threads = d_edge_list_offsets.size() / 2;
    const int grid_size = (nb_threads + block_size - 1) / block_size;
    float* d_values_a = d_values;
    float* d_values_b = d_values_buffer;
    strength = std::max( 0.f, std::min(1.f, strength));
    for(int i = 0; i < nb_iter; i++)
    {
        diffusion_kernel<<<grid_size, block_size>>>
            (d_values_a, d_values_b, d_edge_list.ptr(), d_edge_list_offsets.ptr(), strength, nb_threads);
        CUDA_CHECK_ERRORS();
        Utils::swap_pointers(d_values_a, d_values_b);
    }

    if(nb_iter % 2 == 1){
        // d_vertices[n] = d_tmp_vertices[n]
        copy_arrays<<<grid_size, block_size>>>(d_values_buffer, d_values, nb_threads);
        CUDA_CHECK_ERRORS();
    }
}

// -----------------------------------------------------------------------------

__global__
void fill_index(DA_int array)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < array.size()) array[p] = p;
}

/// Evaluate skeleton potential
__device__
float eval_potential(Skeleton_env::Skel_id skel_id, const Point_cu& p, Vec3_cu& grad)
{
    return Skeleton_env::compute_potential(skel_id, p, grad);
}

// -----------------------------------------------------------------------------

/// Computes the potential at each vertex of the mesh. When the mesh is
/// animated, if implicit skinning is enabled, vertices move so as to match that
/// value of the potential.
__global__
void compute_base_potential(Skeleton_env::Skel_id skel_id,
                            const Point_cu* in_verts,
                            const int nb_verts,
                            float* base_potential,
                            Vec3_cu* base_grad)
{
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_verts)
    {
        float f = eval_potential(skel_id, in_verts[p], base_grad[p]);
        base_potential[p] = f;
    }
}

// -----------------------------------------------------------------------------

__device__
float binary_search(Skeleton_env::Skel_id skel_id,
                        const Ray_cu&r,
                        float t0, float t1,
                        Vec3_cu& grad,
                        float iso)
{
    float t = t0;
    float f0 = eval_potential(skel_id, r(t0), grad);
    float f1 = eval_potential(skel_id, r(t1), grad);

    if(f0 > f1){
        t0 = t1;
        t1 = t;
    }

    Point_cu p;
    for(unsigned short i = 0 ; i < BINARY_SEARCH_STEPS; ++i)
    {
        t = (t0 + t1) * 0.5f;
        p = r(t);
        f0 = eval_potential(skel_id, p, grad);

        if(f0 > iso){
            t1 = t;
            if((f0-iso) < EPSILON) break;
        } else {
            t0 = t;
            if((iso-f0) < EPSILON) break;
        }
    }
    return t;
}

// -----------------------------------------------------------------------------

/// Search for the gradient divergence section
__device__
float binary_search_div(Skeleton_env::Skel_id skel_id,
                            const Ray_cu&r,
                            float t0, float t1,
                            Vec3_cu& grad1,
                            float threshold)
{
    //#define FROM_START
    float t;
    Vec3_cu grad0, grad;
    float f = eval_potential(skel_id, r(t0), grad0);

    Point_cu p;
    for(unsigned short i = 0; i < BINARY_SEARCH_STEPS; ++i)
    {
        t = (t0 + t1) * 0.5f;
        p = r(t);
        f = eval_potential(skel_id, p, grad);

        if(grad.dot(grad0) > threshold)
        {
            t1 = t;
            grad1 = grad;
        }
        else if(grad.dot(grad1) > threshold)
        {
            t0 = t;
            grad0 = grad;
        }
        else
            break;// No more divergence maybe its a false collision ?
    }
    #ifdef FROM_START
    grad = grad0;
    return t0; // if
    #else
    return t;
    #endif
}

// -----------------------------------------------------------------------------

/// transform iso to sfactor
__device__
inline static float iso_to_sfactor(float x, int s)
{
     #if 0
    x = fabsf(x);
    // We compute : 1-(x^c0 - 1)^c1
    // with c0=2 and c1=4 (note: c0 and c1 are 'slopes' at point x=0 and x=1 )
    x *= x; // x^2
    x = (x-1.f); // (x^2 - 1)
    x *= x; // (x^2 - 1)^2
    x *= x; // (x^2 - 1)^4
    return (x > 1.f) ? 1.f : 1.f - x/* (x^2 - 1)^4 */;
    #elif 1
    x = fabsf(x);
    // We compute : 1-(x^c0 - 1)^c1
    // with c0=1 and c1=4 (note: c0 and c1 are 'slopes' at point x=0 and x=1 )
    //x *= x; // x^2
    x = (x-1.f); // (x^2 - 1)
    float res = 1.f;
    for(int i = 0; i < s; i++) res *= x;
    x = res; // (x^2 - 1)^s
    return (x > 1.f) ? 1.f : 1.f - x/* (x^2 - 1)^s */;
    #else
    return 1.f;
    #endif
}

/*
    Ajustement standard avec gradient
*/

/// Move the vertices along a mix between their normals and the joint rotation
/// direction in order to match their base potential at rest position
/// @param d_output_vertices  vertices array to be moved in place.
/// @param d_ssd_interpolation_factor  interpolation weights for each vertices
/// which defines interpolation between ssd animation and implicit skinning
/// 1 is full ssd and 0 full implicit skinning
/// @param do_tune_direction if false use the normal to displace vertices
/// @param gradient_threshold when the mesh's points are fitted they march along
/// the gradient of the implicit primitive. this parameter specify when the vertex
/// stops the march i.e when gradient_threshold < to the scalar product of the
/// gradient between two steps
/// @param full_eval tells is we evaluate the skeleton entirely or if we just
/// use the potential of the two nearest clusters, in full eval we don't update
/// d_vert_to_fit has it is suppossed to be the last pass
__global__
void match_base_potential(Skeleton_env::Skel_id skel_id, 
                          const bool final_pass,
                          const bool smooth_fac_from_iso,
                          Vec3_cu* out_verts,
                          const float* base_potential,
                          Vec3_cu* out_gradient,
                          float* smooth_factors_iso,
                          float* smooth_factors,
                          int* vert_to_fit,
                          const int nb_vert_to_fit,
                          const unsigned short nb_iter,
                          const float gradient_threshold,
                          const float step_length,
                          const bool potential_pit, // TODO: this condition should not be necessary
                          int* d_vert_state,
                          const float smooth_strength,
                          const float collision_depth,
                          const int slope,
                          const bool raphson)
{
#ifdef ENABLE_MARCH
    const int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(thread_idx < nb_vert_to_fit)
    {
        const int p = vert_to_fit[thread_idx];

        // STOP CASE : Vertex already fitted
        if(p == -1) return;

        const float ptl = base_potential[p];

        Point_cu v0 = out_verts[p].to_point();
        Vec3_cu gf0;
        float f0;
        f0 = eval_potential(skel_id, v0, gf0) - ptl;

        if(smooth_fac_from_iso)
            smooth_factors_iso[p] = iso_to_sfactor(f0, slope) * smooth_strength;

        out_gradient[p] = gf0;
        // STOP CASE : Gradient is null we can't know where to march
        if(gf0.norm() <= 0.00001f){
            if(!final_pass) vert_to_fit[thread_idx] = -1;
            return;
        }

        // STOP CASE : Point already near enough the isosurface
        if( fabsf(f0) < EPSILON ){
            if(!final_pass) vert_to_fit[thread_idx] = -1;
            return;
        }

        // Inside we march along the inverted gradient
        // outside along the gradient :
        const float dl = (f0 > 0.f) ? -step_length : step_length;

        Ray_cu r;
        r.set_pos(v0);
        float t = 0.f;

        Vec3_cu  gfi    = gf0;
        float    fi     = f0;
        float    abs_f0 = fabsf(f0);
        Point_cu vi     = v0;

        for(unsigned short i = 0; i < nb_iter; ++i)
        {

            r.set_pos(v0);
            if( raphson ){
                float nm = gf0.norm_squared();
                r.set_dir( gf0 );
                t = dl * abs_f0 / nm;
                //t = t < 0.001f ? dl : t;
            } else {
                #if 1
                    r.set_dir( gf0.normalized() );
                #else
                    if( gf0.dot( c_dir ) > 0.f ) r.set_dir(  c_dir );
                    else                         r.set_dir( -c_dir );

                #endif
                t = dl;
            }

            vi = r(t);
            fi = eval_potential(skel_id, vi, gfi) - ptl;

            // STOP CASE 1 : Initial iso-surface reached
            abs_f0 = fabsf(fi);
            if(raphson && abs_f0 < EPSILON )
            {
                if(!final_pass) vert_to_fit[thread_idx] = -1;
                break;
            }
            else if( fi * f0 <= 0.f)
            {
                t = binary_search(skel_id, r, 0.f, t, gfi, ptl);

                if(!final_pass) vert_to_fit[thread_idx] = -1;
                break;
            }

            // STOP CASE 2 : Gradient divergence
            if( (gf0.normalized()).dot(gfi.normalized()) < gradient_threshold)
            {
                #if 0
                t = binary_search_div(r, -step_length, t, .0,
                                          gtmp, gradient_threshold);
                #endif

                if(!final_pass) vert_to_fit[thread_idx] = -1;

                smooth_factors[p] = smooth_strength;
                break;
            }

            // STOP CASE 3 : Potential pit
            if( ((fi - f0)*dl < 0.f) & potential_pit )
            {
                if(!final_pass) vert_to_fit[thread_idx] = -1;
                smooth_factors[p] = smooth_strength;
                break;
            }

            v0  = vi;
            f0  = fi;
            gf0 = gfi;

            if(gf0.norm_squared() < (0.001f*0.001f)) break;
        }

        const Point_cu res = r(t);
        out_gradient[p] = gfi;
        out_verts[p] = res;
    }
#endif
}

}
// END KERNELS NAMESPACE =======================================================


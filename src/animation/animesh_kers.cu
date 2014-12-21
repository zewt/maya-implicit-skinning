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
#include "dual_quat_cu.hpp"
#include "cuda_current_device.hpp"
#include "color.hpp"
#include "std_utils.hpp"


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

// -----------------------------------------------------------------------------

__global__
void transform_SSD(const Point_cu* in_verts,
                   const Vec3_cu* in_normals,
                   int nb_verts,
                   Vec3_cu* out_verts,
                   Vec3_cu* out_normals,
                   const Transfo* transfos,
                   const float* weights,
                   const int* joints,
                   const int* jpv)
{
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_verts )
    {
        // compute vertex new position
        const int st_j = jpv[2*p  ]; // First joint idx in d_transform
        const int nb_j = jpv[2*p+1]; // nb joints influencing the vertex
        Transfo t;
        t = ( nb_j > 0) ? transfos[ joints[st_j] ] * weights[st_j] :
                          Transfo::identity();

        for(int j = st_j + 1; j < (st_j + nb_j); j++)
        {
            const int   k = joints [j];
            const float w = weights[j];
            t = t + transfos[k] * w;
        }

        // Compute animated position
        Vec3_cu vi = (t * in_verts[p]).to_vector();
        out_verts [p] = vi;
        // Compute animated normal
        out_normals[p] = t.fast_invert().transpose() * in_normals[p];
    }
}

// -----------------------------------------------------------------------------

__global__
void transform_dual_quat( const Point_cu* in_verts,
                          const Vec3_cu* in_normals,
                          int nb_verts,
                          Vec3_cu* out_verts,
                          Vec3_cu* out_normals,
                          const Dual_quat_cu* dual_quat,
                          const float* weights,
                          const int* joints,
                          const int* jpv)
{
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_verts)
    {
        // compute vertex new position
        //Vec3_cu axis = {0.f, 0.f, 0.f};

        const int st_j = jpv[2*p  ];
        const int nb_j = jpv[2*p+1];

        int   k0 = -1;
        float w0 = 0.f;
        Dual_quat_cu dq_blend;
        Quat_cu q0;

        if(nb_j != 0)
        {
            k0 = joints [st_j];
            w0 = weights[st_j];
        }else
            dq_blend = Dual_quat_cu::identity();

        if(k0 != -1) dq_blend = dual_quat[k0] * w0;

        int pivot = k0;

        q0 = dual_quat[pivot].rotation();

        for(int j = st_j+1; j < st_j + nb_j; j++)
        {
            const int k = joints [j];
            float w = weights[j];
            const Dual_quat_cu& dq = (k == -1) ? Dual_quat_cu::identity() : dual_quat[k];

            if( dq.rotation().dot( q0 ) < 0.f )
                w *= -1.f;

            dq_blend = dq_blend + dq * w;
        }

        // Compute animated position
        Vec3_cu vi = dq_blend.transform( in_verts[p] ).to_vector();
        out_verts [p] = vi;
        // Compute animated normal
        out_normals[p] = dq_blend.rotate( in_normals[p] );
    }
}

// -----------------------------------------------------------------------------

__global__
void lerp_kernel(const int* vert_to_fit,
                 const Vec3_cu* verts_0,
                 const Vec3_cu* verts_1,
                 float* lerp_factor,
                 Vec3_cu* out_verts,
                 int nb_verts)
{
    const int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(thread_idx < nb_verts)
    {
        const int p = vert_to_fit[thread_idx];

        const float f = lerp_factor[p];
        out_verts[p] = verts_0[p] * (1.f - f) + verts_1[p] * f;
    }
}

// -----------------------------------------------------------------------------

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

__device__ Vec3_cu
compute_normal_quad(const Mesh::PrimIdx& pi, const Vec3_cu* prim_vertices) {
    const Point_cu va = Convs::to_point(prim_vertices[pi.a]);
    const Point_cu vb = Convs::to_point(prim_vertices[pi.b]);
    const Point_cu vc = Convs::to_point(prim_vertices[pi.c]);
    const Point_cu vd = Convs::to_point(prim_vertices[pi.d]);
    Vec3_cu vab = (vb - va);
    Vec3_cu vbc = (vc - vb);
    Vec3_cu vcd = (vd - vc);
    Vec3_cu vda = (va - vb);

    return ((vda - vbc).cross(vab - vcd)).normalized();
    //return Vec3_cu(1,1,1).normalized();
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
    if(p < n){
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
}

// -----------------------------------------------------------------------------
/*
void
compute_unpacked_normals_tri_debug_cpu(const int* d_faces,
                                       Device::Array< Mesh::PrimIdxVertices> d_piv,
                                       int nb_faces,
                                       const Vec3_cu* vertices,
                                       Device::Array<Vec3_cu> d_unpacked_normals,
                                       int unpack_factor,
                                       int blockDim,
                                       int gridDim
                                       )
{
    HA_int faces(nb_faces*3);
    mem_cpy_dth(faces.ptr(), d_faces, faces.size());
    Host::Array< PrimIdxVertices > piv(d_piv.size());
    piv.copy_from(d_piv);

    HA_Vec3_cu unpacked_normals(d_unpacked_normals.size());
    unpacked_normals.copy_from(d_unpacked_normals);

    for(int blockIdx = 0; blockIdx < blockDim; blockIdx++ )
        for(int threadIdx = 0; threadIdx < gridDim; threadIdx++ )
        {
            int n = nb_faces;
            int p = blockIdx * blockDim + threadIdx;
            if(p < n){
                Mesh::PrimIdx pidx;
                pidx.a = faces[3*p];
                pidx.b = faces[3*p + 1];
                pidx.c = faces[3*p + 2];
                Mesh::PrimIdxVertices pivp = piv[p];
                Vec3_cu nm = {0.f,0.f,0.f};// = compute_normal_tri(pidx, vertices);//////////
                int ia = pidx.a * unpack_factor + pivp.ia;
                int ib = pidx.b * unpack_factor + pivp.ib;
                int ic = pidx.c * unpack_factor + pivp.ic;
                unpacked_normals[ia] = nm;
                unpacked_normals[ib] = nm;
                unpacked_normals[ic] = nm;
            }
        }
    d_unpacked_normals.copy_from(unpacked_normals);
}
*/
// -----------------------------------------------------------------------------

__global__ void
compute_unpacked_normals_quad(const int* faces,
                              Device::Array< Mesh::PrimIdxVertices> piv,
                              int nb_faces,
                              int piv_offset,
                              const Vec3_cu* vertices,
                              Device::Array<Vec3_cu> unpacked_normals,
                              int unpack_factor){
    int n = nb_faces;
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n)
    {
        Mesh::PrimIdx pidx;
        pidx.a = faces[4*p];
        pidx.b = faces[4*p + 1];
        pidx.c = faces[4*p + 2];
        pidx.d = faces[4*p + 3];
        Mesh::PrimIdxVertices pivp = piv[p + piv_offset];
        Vec3_cu nm = compute_normal_quad(pidx, vertices);
        int ia = pidx.a * unpack_factor + pivp.ia;
        int ib = pidx.b * unpack_factor + pivp.ib;
        int ic = pidx.c * unpack_factor + pivp.ic;
        int id = pidx.d * unpack_factor + pivp.id;
        unpacked_normals[ia] = nm;
        unpacked_normals[ib] = nm;
        unpacked_normals[ic] = nm;
        unpacked_normals[id] = nm;
        //unpacked_normals[p] = Vec3_cu(pivp.ia, pivp.ib, pivp.ic);
    }

}

// -----------------------------------------------------------------------------

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
                     const int* quad,
                     Device::Array<Mesh::PrimIdxVertices> piv,
                     int nb_tri,
                     int nb_quad,
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

    const int nb_threads_compute_quad = nb_quad;
    const int grid_size_compute_quad = (nb_threads_compute_quad + block_size - 1) / block_size;

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
    if(nb_quad > 0){
        compute_unpacked_normals_quad<<< grid_size_compute_quad, block_size>>>
                                     (quad,
                                      piv,
                                      nb_quad,
                                      nb_tri,
                                      vertices,
                                      unpacked_normals,unpack_factor);
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

__device__ Vec3_cu
compute_tangent_quad(const Mesh::PrimIdx& pi, const Vec3_cu* prim_vertices) {
    const Point_cu va = Convs::to_point(prim_vertices[pi.a]);
    const Point_cu vb = Convs::to_point(prim_vertices[pi.b]);
    const Point_cu vc = Convs::to_point(prim_vertices[pi.c]);
    const Point_cu vd = Convs::to_point(prim_vertices[pi.d]);
    Vec3_cu vab = (vb - va);
    Vec3_cu vbc = (vc - vb);
    Vec3_cu vcd = (vd - vc);
    Vec3_cu vda = (va - vb);

    return ((vda - vbc).cross(vab - vcd)).normalized();
    //return Vec3_cu(1,1,1).normalized();
}

// -----------------------------------------------------------------------------

/** Assign the normal of each face to each of its vertices
  */
__global__ void
compute_unpacked_tangents_tri(const int* faces,
                              const int* unpacked_faces,
                              Device::Array<Mesh::PrimIdxVertices> piv,
                              int nb_faces,
                              const Vec3_cu* vertices,
                              const float* tex_coords,
                              Device::Array<Vec3_cu> unpacked_tangents,
                              int unpack_factor)
{
    int n = nb_faces;
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        Mesh::PrimIdx pidx;
        pidx.a = faces[3*p    ];
        pidx.b = faces[3*p + 1];
        pidx.c = faces[3*p + 2];
        Mesh::PrimIdx upidx;
        upidx.a = unpacked_faces[3*p    ];
        upidx.b = unpacked_faces[3*p + 1];
        upidx.c = unpacked_faces[3*p + 2];
        Mesh::PrimIdxVertices pivp = piv[p];
        Vec3_cu nm = compute_tangent_tri(pidx, upidx, vertices, tex_coords);
        int ia = pidx.a * unpack_factor + pivp.ia;
        int ib = pidx.b * unpack_factor + pivp.ib;
        int ic = pidx.c * unpack_factor + pivp.ic;
        unpacked_tangents[ia] = nm;
        unpacked_tangents[ib] = nm;
        unpacked_tangents[ic] = nm;
    }
}

// -----------------------------------------------------------------------------

__global__ void
compute_unpacked_tangents_quad(const int* faces,
                               const int* unpacked_faces,
                               Device::Array< Mesh::PrimIdxVertices> piv,
                               int nb_faces,
                               int piv_offset,
                               const Vec3_cu* vertices,
                               const float* tex_coords,
                               Device::Array<Vec3_cu> unpacked_tangents,
                               int unpack_factor)
{
    int n = nb_faces;
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        Mesh::PrimIdx pidx;
        pidx.a = faces[4*p];
        pidx.b = faces[4*p + 1];
        pidx.c = faces[4*p + 2];
        pidx.d = faces[4*p + 3];
        Mesh::PrimIdxVertices pivp = piv[p + piv_offset];
        Vec3_cu nm = compute_tangent_quad(pidx, vertices);
        int ia = pidx.a * unpack_factor + pivp.ia;
        int ib = pidx.b * unpack_factor + pivp.ib;
        int ic = pidx.c * unpack_factor + pivp.ic;
        int id = pidx.d * unpack_factor + pivp.id;
        unpacked_tangents[ia] = nm;
        unpacked_tangents[ib] = nm;
        unpacked_tangents[ic] = nm;
        unpacked_tangents[id] = nm;
        //unpacked_normals[p] = Vec3_cu(pivp.ia, pivp.ib, pivp.ic);
    }

}

// -----------------------------------------------------------------------------

void compute_tangents(const int* tri,
                      const int* quad,
                      const int* unpacked_tri,
                      const int* unpacked_quad,
                      Device::Array<Mesh::PrimIdxVertices> piv,
                      int nb_tri,
                      int nb_quad,
                      const Vec3_cu* vertices,
                      const float* tex_coords,
                      Device::Array<Vec3_cu> unpacked_tangents,
                      int unpack_factor,
                      Vec3_cu* out_tangents)
{

    const int block_size = 512;
    const int nb_threads_clean = unpacked_tangents.size();
    const int grid_size_clean = (nb_threads_clean + block_size - 1) / block_size;
    const int nb_threads_pack = unpacked_tangents.size() / unpack_factor;
    const int grid_size_pack = (nb_threads_pack + block_size - 1) / block_size;

    const int nb_threads_compute_tri = nb_tri;
    const int grid_size_compute_tri = (nb_threads_compute_tri + block_size - 1) / block_size;

    const int nb_threads_compute_quad = nb_quad;
    const int grid_size_compute_quad = (nb_threads_compute_quad + block_size - 1) / block_size;

    CUDA_CHECK_KERNEL_SIZE(block_size, grid_size_clean);
    clean_unpacked_normals<<< grid_size_clean, block_size>>>(unpacked_tangents);

    CUDA_CHECK_ERRORS();

    if(nb_tri > 0){
        compute_unpacked_tangents_tri<<< grid_size_compute_tri, block_size>>>
                                   (tri,
                                    unpacked_tri,
                                    piv,
                                    nb_tri,
                                    vertices,
                                    tex_coords,
                                    unpacked_tangents,unpack_factor);
        CUDA_CHECK_ERRORS();
    }
    if(nb_quad > 0){
        assert(false);
        // TODO: handle quads
        compute_unpacked_tangents_quad<<< grid_size_compute_quad, block_size>>>
                                     (quad,
                                      unpacked_quad,
                                      piv,
                                      nb_quad,
                                      nb_tri,
                                      vertices,
                                      tex_coords,
                                      unpacked_tangents,unpack_factor);
        CUDA_CHECK_ERRORS();
    }

    pack_normals<<< grid_size_pack, block_size>>>(unpacked_tangents,
                                                  unpack_factor,
                                                  out_tangents);
    CUDA_CHECK_ERRORS();
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
void unpack_vert_and_normals(const Vec3_cu* packed_vert,
                             const Vec3_cu* packed_normals,
                             const Vec3_cu* packed_tangents,
                             const Mesh::Packed_data* packed_vert_map,
                             Vec3_cu* unpacked_vert,
                             Vec3_cu* unpacked_normals,
                             Vec3_cu* unpacked_tangents,
                             int nb_vert)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_vert)
    {
        Vec3_cu pv = packed_vert[p];
        Vec3_cu pn = packed_normals[p];
        Vec3_cu pt;
        if(unpacked_tangents != 0)
            pt = packed_tangents[p];

        Mesh::Packed_data d = packed_vert_map[p];
        int idx = d.idx_data_unpacked;
        for(int i = 0; i < d.nb_ocurrence; i++)
        {
            unpacked_vert    [idx+i] = pv;
            unpacked_normals [idx+i] = pn;
            if(unpacked_tangents != 0)
                unpacked_tangents[idx+i] = pt;
        }
    }
}

// -----------------------------------------------------------------------------

__global__
void fill_index(DA_int array)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < array.size()) array[p] = p;
}

// -----------------------------------------------------------------------------

}// END KERNELS NAMESPACE ======================================================

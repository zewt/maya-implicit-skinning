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
#ifndef ANIMATED_MESH__
#define ANIMATED_MESH__

// -----------------------------------------------------------------------------

#include "animesh_enum.hpp"
#include "cuda_utils.hpp"
#include "mesh.hpp"

// -----------------------------------------------------------------------------

#include <map>
#include <vector>

// Forward def -----------------------------------------------------------------
//#include "skeleton.hpp"
#include "tree_cu_type.hpp"
#include "bone.hpp"
struct Skeleton;
// End Forward def -------------------------------------------------------------

/** @brief Class of animated meshes

    As the joints of the skeleton rotates, the mesh is deformed accordingly.
*/

struct Animesh{
public:

    // -------------------------------------------------------------------------
    /// @name Inner class
    // -------------------------------------------------------------------------



    /// @warning The input mesh 'm_' vertices memory layout might be changed
    /// for optimization. Yet we garantee the layout is same in Animesh as
    /// in Mesh
    Animesh(const Mesh *m_, const Skeleton *s_);

    ~Animesh();

    /// Computes the potential at each vertex of the mesh. When the mesh is
    /// animated, if implicit skinning is enabled, vertices move so as to match
    /// that value of the potential.
    void update_base_potential();

    // Read and write the base potential (and gradient).
    void get_base_potential(std::vector<float> &pot, std::vector<Vec3_cu> &grad) const;
    void set_base_potential(const std::vector<float> &pot, const std::vector<Vec3_cu> &grad);

    /// Transform the vertices of the mesh given the rotation at each bone.
    /// Transformation is computed from the initial position of the mesh
    /// @param type specify the technic used to compute vertices deformations
    void transform_vertices();

    // -------------------------------------------------------------------------
    /// @name Getter & Setters
    // -------------------------------------------------------------------------

    /// Fill the vector as in the imported file order
    void get_anim_vertices_aifo(std::vector<Point_cu>& anim_vert);

    // Return the number of vertices in the mesh.  Calls to copy_vertices must have the
    // same number of vertices.
    int get_nb_vertices() const { return d_input_vertices.size(); }

    // Copy the given vertices into the mesh.
    void copy_vertices(const std::vector<Vec3_cu> &vertices);

    inline void set_smooth_factor(int i, float val){
        d_input_smooth_factors.set(i, val);
    }

    void set_smoothing_weights_diffusion_iter(int nb_iter){
        diffuse_smooth_weights_iter = nb_iter;
    }
    void set_smoothing_iter (int nb_iter ) { smoothing_iter = nb_iter;   }
    void set_smooth_mesh    (bool state  ) { do_smooth_mesh = state;     }
    void set_local_smoothing(bool state  ) { do_local_smoothing = state; }
    void set_smooth_force_a (float alpha ) { smooth_force_a = alpha;     }
    void set_smooth_force_b (float beta  ) { smooth_force_b = beta;      }
    void set_smooth_smear   (float v     ) { smooth_smear   = v;         }

    void set_smoothing_type (EAnimesh::Smooth_type type ) {
        mesh_smoothing = type;
    }

#if !defined(NO_CUDA)
    inline const Cuda_utils::DA_Vec3_cu& get_rot_axis() const { return d_rot_axis; }
    inline const Cuda_utils::DA_Vec3_cu& get_gradient() const { return d_gradient; }
#endif

    const Mesh*     get_mesh() const { return _mesh; }
    const Skeleton* get_skel() const { return _skel; }

private:
    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------

    /// Tangential smoothing on GPU
    /// @param d_vertices vertices to be processed in place
    /// @param d_vertices_prealloc allocated buffer of vertices as so the method
    /// does not allocate temporary space at each call
    /// @param d_normals normals after smoothing
    /// @param nb_iter number of iteration for smoothing the mesh.
    /// N.B : Even numbers of iteratons are faster than odd.
    void tangential_smooth(float* factors,
                           Vec3_cu* d_vertices,
                           Vec3_cu* d_vertices_prealloc,
                           Vec3_cu* d_normals,
                           int nb_iter);

    /// make the mesh smooth with the smoothing technique specified by
    /// mesh_smoothing
    void smooth_mesh(Vec3_cu* output_vertices,
                     Vec3_cu* output_normals,
                     float* factors,
                     int nb_iter,
                     bool local_smoothing);


    void conservative_smooth(Vec3_cu* output_vertices,
                             Vec3_cu* buff,
                             const Cuda_utils::DA_int& d_vert_to_fit,
                             int nb_vert_to_fit,
                             int nb_iter,
                             bool use_vert_to_fit = true);

    /// Compute normals in 'normals' and the vertices position in 'vertices'
    void compute_normals(const Vec3_cu* vertices, Vec3_cu* normals);

    void fit_mesh(int nb_vert_to_fit,
                  int* d_vert_to_fit,
                  bool final_pass,
                  bool smooth_fac_from_iso,
                  Vec3_cu *d_vertices,
                  int nb_steps, float smooth_strength);

    /// diffuse values over the mesh on GPU
    void diffuse_attr(int nb_iter, float strength, float* attr);

    int pack_vert_to_fit(Cuda_utils::Host::Array<int>& in,
                         Cuda_utils::Host::Array<int>& out,
                         int last_nb_vert_to_fit);

    /// Pack negative index in 'd_vert_to_fit' (done on gpu)
    /// @param d_vert_to_fit list of vertices to fit negative indices are to be
    /// eliminated and indices regrouped to the begining of this array
    /// @param buff_prefix_sum intermediate array filled with the prefix sum of
    /// 'd_vert_to_fit' where negative indices are considered as zeros positives
    /// as ones. buff_prefix_sum.size() == 1+d_vert_to_fit.size()
    /// @param packed_vert_to_fit result of the packing of 'd_vert_to_fit'.
    /// packed_vert_to_fit contains the positive indices regrouped at its
    /// beginning. packed_vert_to_fit.size() == d_vert_to_fit.size()
    /// @return the number of vertices to fit (i.e number of index >= 0)
    /// its the size of packed_vert_to_fit which holds the positive indices of
    /// d_vert_to_fit
    int pack_vert_to_fit_gpu(
            Cuda_utils::Device::Array<int>& d_vert_to_fit,
            Cuda_utils::Device::Array<int>& buff_prefix_sum,
            Cuda_utils::Device::Array<int>& packed_vert_to_fit,
            int nb_vert_to_fit);

    /// Copy the attributes of 'a_mesh' into the attributes of the animated
    /// mesh in device memory
    void copy_mesh_data(const Mesh& a_mesh);

    /// Compute the mean value coordinates (mvc) of every vertices in rest pose
    /// @note : in some special cases the sum of mvc will be exactly equal to
    /// zero. This will have to be dealt with properly  when using them. For
    /// instance when smoothing we will have to check that.
    /// Cases :
    /// - Vertex is a side of the mesh
    /// - one of the mvc coordinate is negative.
    /// (meaning the vertices is outside the polygon the mvc is expressed from)
    /// - Normal of the vertices has norm == zero
    void compute_mvc();

    /// Allocate and initialize 'd_vert_to_fit' and 'd_vert_to_fit_base'.
    /// For instance lonely vertices are not fitted with the implicit skinning.
    void init_vert_to_fit();

    void init_smooth_factors(Cuda_utils::DA_float& d_smooth_factors);

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    /// The mesh 'm' is deformed after each call of transform_vertices().
    /// deformation is computed from the initial position of the mesh stored in
    /// d_input_vertices. The mesh buffer objects attributes defines the animated
    /// mesh
    const Mesh *_mesh;
    const Skeleton *_skel;

    EAnimesh::Smooth_type mesh_smoothing;

    bool do_smooth_mesh;
    bool do_local_smoothing;
    bool do_interleave_fitting;

    /// Smoothing strength after animation

    int smoothing_iter;
    int diffuse_smooth_weights_iter;
    float smooth_force_a; ///< must be between [0 1]
    float smooth_force_b; ///< must be between [0 1] only for humphrey smoothing
    float smooth_smear;   ///< between [-1 1]

    /// Smoothing weights associated to each vertex
    Cuda_utils::Device::Array<float> d_input_smooth_factors;
    /// Animated smoothing weights associated to each vertex
    Cuda_utils::Device::Array<float> d_smooth_factors_conservative;

    /// Smooth factor at each vertex depending on SSD
    Cuda_utils::Device::Array<float> d_smooth_factors_laplacian;

    /// Initial vertices in their "resting" position. animation is compute with
    /// these points
    Cuda_utils::Device::Array<Point_cu> d_input_vertices;

    /// Store for each edge its length
    /// @note to look up this list you need to use 'd_edge_list_offsets'
    Cuda_utils::Device::Array<float> d_edge_lengths;

    /// Store for each edge the mean value coordinate of the vertex according to
    /// the thirst ring of neighborhood. (Mean value coordinates are barycentric
    /// coordinates where we can express v as the sum of its neighborhood
    /// neigh_i. v = sum from 0 to nb_neigh { mvc_i * neigh_i } )
    /// @note to look up this list you need to use 'd_edge_list_offsets'
    Cuda_utils::Device::Array<float> d_edge_mvc;

    /// Stores in which state a vertex is when fitted into the implicit surface
    Cuda_utils::Device::Array<EAnimesh::Vert_state> d_vertices_state;
    /// Colors associated to the enum field EAnimesh::Vert_state
    Cuda_utils::Device::Array<float4> d_vertices_states_color;

    /// Animated vertices in their final position.
    Cuda_utils::Device::Array<Point_cu>  d_output_vertices;

    /// Gradient of the implicit surface at each vertices when animated
    Cuda_utils::Device::Array<Vec3_cu> d_gradient;

    /// triangle index in device mem. We don't use the mesh's vbos because
    /// there are different from the mesh's real topology as some of the vertices
    /// are duplicated for rendering because of textures
    Cuda_utils::Device::Array<int> d_input_tri;

    /// List of first ring neighborhoods for a vertices, this list has to be
    /// read with the help of d_edge_list_offsets[] array @see d_edge_list_offsets
    Cuda_utils::Device::Array<int> d_edge_list;
    /// Table of indirection in order to read d_edge_list[] array.
    /// For the ith vertex d_edge_list_offsets[2*ith] gives the offset from
    /// which d_edge_list as to be read and d_edge_list_offsets[2*ith+1] gives
    /// the number of neighborhood for the ith vertex.
    Cuda_utils::Device::Array<int> d_edge_list_offsets;

    /// Base potential associated to the ith vertex (i.e in rest pose of skel)
    Cuda_utils::Device::Array<float> d_base_potential;

    /// Base gradient associated to the ith vertex (i.e in rest pose of skel)
    Cuda_utils::Device::Array<Vec3_cu> d_base_gradient;

    /// Buffer used to compute normals on GPU. this array holds normals for each
    /// face. d_unpacked_normals[vert_id*nb_max_face_per_vert + ith_face_of_vert]
    /// == normal_at_vert_id_for_its_ith_face
    Cuda_utils::Device::Array<Vec3_cu> d_unpacked_normals;
    /// same as 'd_unpacked_normals' but with tangents
    Cuda_utils::Device::Array<Vec3_cu> d_unpacked_tangents;
    /// ?
    Cuda_utils::Device::Array<Mesh::PrimIdxVertices> d_piv;

    /// Vector representing the rotation axis of the nearest joint for each
    /// vertex. d_rot_axis[vert_id] == vec_rotation_axis
    Cuda_utils::Device::Array<Vec3_cu>  d_rot_axis;

    // -------------------------------------------------------------------------
    /// @name SSD Weights
    // -------------------------------------------------------------------------

    /// Table of indirection which associates for the ith vertex its list of
    /// weights and joint IDs.
    /// For the ith vertex d_jpv gives for (ith*2) the starting index in
    /// d_joints. The (ith*2+1) element gives the number of joint/
    /// weigths associated to the vertex in d_joints.
    Cuda_utils::Device::Array<int> d_jpv;

    // -------------------------------------------------------------------------
    /// @name CLUSTER
    // -------------------------------------------------------------------------

    typedef Skeleton_env::DBone_id DBone_id;

    // END CLUSTER -------------------------------------------------------------

    /// Vertices behind a joint when it flex
    Cuda_utils::Device::Array<bool> d_rear_verts;

    // -------------------------------------------------------------------------
    /// @name Pre allocated arrays to store intermediate results of the mesh
    // -------------------------------------------------------------------------
    /// @{
    Cuda_utils::Host::Array<Vec3_cu>    h_vert_buffer;
    Cuda_utils::Device::Array<Vec3_cu>  d_vert_buffer;
    Cuda_utils::Device::Array<Vec3_cu>  d_vert_buffer_2;
    Cuda_utils::Device::Array<float>    d_vals_buffer;

    Cuda_utils::Device::Array<int>      d_vert_to_fit;
    Cuda_utils::Device::Array<int>      d_vert_to_fit_base;
    Cuda_utils::Device::Array<int>      d_vert_to_fit_buff_scan;
    Cuda_utils::Device::Array<int>      d_vert_to_fit_buff;

    Cuda_utils::Host::Array<int>        h_vert_to_fit_buff;
    /// @}
};
// END ANIMATEDMESH CLASS ======================================================

#endif // ANIMATED_MESH__

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
#include "gl_mesh.hpp"
#include "mesh.hpp"
#include "camera.hpp"

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

    @warning the class might reorder the mesh vertices when creating the
    Animesh object instance. Methods output and input in the old order
    are suffixed 'aifo' that is to say 'as in imported file order'. Methods not
    marked are suppose to use the new order.
    A mapping between old and new order is stored in 'vmap_old_new' :
    vmap_old_new[old_idx] == new_idx
*/

struct Animesh{
public:

    // -------------------------------------------------------------------------
    /// @name Inner class
    // -------------------------------------------------------------------------

    /// @brief base class to find HRBF samples on the mesh given a bone
    class HRBF_sampling {
    public:
        HRBF_sampling( Animesh* am ) :
            _bone_id(-1),
            _factor_siblings(false),
            _am(am)
        { }

        virtual ~HRBF_sampling(){ }

        /// Given the mesh and the defined bone _bone_id samples the mesh
        /// surface.
        virtual void sample(std::vector<Vec3_cu>& out_verts,
                            std::vector<Vec3_cu>& out_normals) = 0;

        /// factor the samples if '_factor_siblings' is true. This returns
        /// The samples factored in the first bone for bones in the same level
        /// of the skeleton tree. The other children don't have samples
        void factor_samples(std::vector<int>& vert_ids,
                            std::vector<Vec3_cu>& verts,
                            std::vector<Vec3_cu>& normals);

        /// Eliminates samples too far from the bone using parameters "_jmax"
        /// and "_pmax"
        /// @warning reset skeleton before using this
        void clamp_samples(std::vector<int>& vert_ids,
                           std::vector<Vec3_cu>& verts,
                           std::vector<Vec3_cu>& normals);

        /// Bone to base the heuristic on
        int _bone_id;

        /// consider bones with multiple children has a single bone
        /// Samples will be added to the first child and other bone on the same
        /// level of the skeleton tree will be ignored.
        bool _factor_siblings;

        float _jmax; ///< percentage max dist from joint (range [-1 1])
        float _pmax; ///< percentage max dist from joint parent (range [-1 1])
        float _fold; ///< threshold scalar product dir projection/mesh normal

        Animesh* _am;
    };

    // -------------------------------------------------------------------------

    /// @brief sampling the mesh using an adhoc heuristic
    /// based on the mesh vertices
    class Adhoc_sampling : public HRBF_sampling {
    public:

        Adhoc_sampling( Animesh* am ) :
            HRBF_sampling(am),
            _mind(0.f)
        {}

        void sample(std::vector<Vec3_cu>& out_verts,
                    std::vector<Vec3_cu>& out_normals);

        float _mind; ///< minimal distance between two samples
    };

    /// @brief sampling the mesh surface with a poisson disk strategy
    class Poisson_disk_sampling : public HRBF_sampling {
    public:

        Poisson_disk_sampling( Animesh* am ) :
            HRBF_sampling(am),
            _mind(0.f),
            _nb_samples(0)
        {}

        void sample(std::vector<Vec3_cu>& out_verts,
                    std::vector<Vec3_cu>& out_normals);

        ///< minimal distance between two samples (if == 0 e use _nb_samples)
        float _mind;
        int _nb_samples;
    };

    // -------------------------------------------------------------------------

    /// @brief some parameters to give when painting the mesh
    /// @see paint()
    struct Paint_setup{
        bool _rest_pose;      ///< Use verts in rest pose
        bool _backface_cull;  ///< Paint only front face
        int _brush_radius;    ///< Brush radius
        int _x, _y;           ///< Brush center
        float _val;           ///< Value to paint (depends on the painting mode)
    };

    // -------------------------------------------------------------------------

    /*-------*/
    /*       */
    /*-------*/

    /// @warning The input mesh 'm_' vertices memory layout might be changed
    /// for optimization. Yet we garantee the layout is same in Animesh as
    /// in Mesh
    Animesh(Mesh* m_, Skeleton* s_);

    ~Animesh();

    /// Computes the potential at each vertex of the mesh. When the mesh is
    /// animated, if implicit skinning is enabled, vertices move so as to match
    /// that value of the potential.
    void update_base_potential();

    /// Transform the vertices of the mesh given the rotation at each bone.
    /// Transformation is computed from the initial position of the mesh
    /// @param type specify the technic used to compute vertices deformations
    void transform_vertices(EAnimesh::Blending_type type);

    /// Draw the mesh for the current position of the skeleton
    /// @deprecated
    void draw(bool use_color_array = true, bool use_point_color = false) const;

    /// @deprecated
    void draw_rest_pose(bool use_color_array = true, bool use_point_color = false) const;

    /// @deprecated
    void draw_points_rest_pose() const;

    /// Paint the attribute defined by 'mode'. Painting is done by projecting
    /// mesh's vertices with cuda
    void paint(EAnimesh::Paint_type mode,
               const Paint_setup& setup,
               const Camera& cam);

    // -------------------------------------------------------------------------
    /// @name HRBF
    // -------------------------------------------------------------------------

    /// replace the ith bone samples with the samples in 'nodes' and 'n_nodes'
    /// This converts the bone to an Hrbf_bone
    void update_bone_samples(Bone::Id id_bone,
                             const std::vector<Vec3_cu>& nodes,
                             const std::vector<Vec3_cu>& n_nodes);

    float compute_nearest_vert_to_bone(int bone_id);

    /// Compute caps at the tip of the bone to close the hrbf
    /// @param use_parent_dir: add the cap following the parent direction and
    /// not the direction of 'bone_id'
    void compute_pcaps(int bone_id,
                       bool use_parent_dir,
                       std::vector<Vec3_cu>& out_verts,
                       std::vector<Vec3_cu>& out_normals);

    /// Compute caps at the tip of the bone to close the hrbf
    void compute_jcaps(int bone_id,
                       std::vector<Vec3_cu>& out_verts,
                       std::vector<Vec3_cu>& out_normals);

    /// Update the mesh clusters according to the skeleton
    /// @param type : choose between to clusterisation algorithm either with
    /// a fast euclidean distance or a (slow) geodesic inside the volume of
    /// the mesh
    /// @param n_voxels : if using geodesic distance this specify the number
    /// of voxels used for the larger side of the mesh's bb
    void clusterize(int n_voxels = 25);

    // -------------------------------------------------------------------------
    /// @name Import export
    /// @deprecated You should use the parsers module
    // -------------------------------------------------------------------------

    /// Reads the weight file that define how each joint of the skeleton
    /// deforms each vertex of the mesh
    /// @deprecated
    void read_weights_from_file(const char* filename, bool file_has_commas);

    /// Export the ssd weights associated to each vertices
    /// @deprecated
    void export_weights(const char* filename);

    /// Export the mesh for the current position of the skeleton
    /// (i.e mesh is export from the current vbo used to draw it)
    /// @deprecated
    void export_off(const char* filename) const;

    // -------------------------------------------------------------------------
    /// @name SSD Related
    // -------------------------------------------------------------------------

    /// Initialize the interpolation weights that define the proportion of
    /// implicit skinning vs ssd. Each vertex is associated to a weight equals
    /// to 0 (full implicit skinning) when it is inside the implicit primitive
    /// or 1 (full SSD) when it is outside. These weights are then diffused
    /// along the mesh to smoothen transition between SSD animation and IS.
    void init_ssd_interpolation_weights();

    /// Set the weight of the ith vertex associated to the if joint,
    /// the value is clamped between [0, 1], and the value associated to the
    /// other joints are normalized.
    void set_ssd_weight(int id_vertex, int id_joint, float weight);
    void init_rigid_ssd_weights();

    float get_ssd_weight(int id_vertex, int id_joint);

    /// @param weights Array vector of ssd weights per vertices and per bone:
    /// vec[vert_id][...].first == bone_id, vec[vert_id][...].second == weight
    /// use case:
    /** @code
        std::vector<std::map<int, float> > weights;
        get_ssd_weights(weights);
        std::map<int, float>& map = weights[index_vert];
        std::map<int, float>::iterator it;
        for(it = map.begin(); it != map.end(); ++it){
            const int   bone_id = it->first;
            const float weight  = it->second;
        }
        @endcode
    */
    void get_ssd_weights(std::vector<std::map<int, float> >& weights);

    /// copy device ssd weights on host side
    void update_host_ssd_weights();

    void set_ssd_weights(const std::vector<std::map<int, float> >& weights);

    void update_device_ssd_weights();

    // -------------------------------------------------------------------------
    /// @name Getter & Setters
    // -------------------------------------------------------------------------

    /// Fill the vector as in the imported file order
    void get_anim_vertices_aifo(std::vector<float>& anim_vert);

    /// Set bone type
    void set_bone_type(int id, int bone_type);

    void get_ssd_lerp(std::vector<float>& ssd_to_is_lerp) const{
        const Cuda_utils::DA_float& tab = d_ssd_interpolation_factor;
        ssd_to_is_lerp.resize( tab.size() );
        Cuda_utils::mem_cpy_dth(&(ssd_to_is_lerp[0]), tab.ptr(), tab.size() );
    }

    void set_ssd_is_lerp(const std::vector<float>& vec) {
        assert( (int)vec.size() == d_ssd_interpolation_factor.size());
        d_ssd_interpolation_factor.copy_from( vec );
    }

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

    void set_enable_update_base_potential(bool s){ do_update_potential = s; }

    /// Switch between implicit skinning and basic ssd skinning
    inline void set_implicit_skinning  ( bool s) { do_implicit_skinning = s;                     }

    inline const Cuda_utils::DA_Vec3_cu& get_ssd_normals() const { return d_ssd_normals; }
    inline const Cuda_utils::DA_Vec3_cu& get_rot_axis() const { return d_rot_axis; }
    inline const Cuda_utils::DA_Vec3_cu& get_gradient() const { return d_gradient; }

    inline int get_nearest_bone(int vert_idx){
        return h_vertices_nearest_bones[vert_idx];
    }

    float get_junction_radius(int bone_id);

    // TO be deleted and moved in the ctrl
    void set_junction_radius(int bone_id, float rad);

    void set_flip_propagation(int vid, bool s) { d_flip_propagation.set(vid, s); }

    void reset_flip_propagation();

    GlBuffer_obj* get_vbo_rest_pose(){
        return _vbo_input_vert;
    }

    GlBuffer_obj* get_nbo_rest_pose(){
        return _nbo_input_normal;
    }

    const Mesh*     get_mesh() const { return _mesh; }
    const Skeleton* get_skel() const { return _skel; }

    Skeleton* get_skel(){ return _skel; }

    const Gl_mesh_quad* get_vox_mesh() const { return _vox_mesh; }

private:
    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------

    /// Compute the nearest cluster of bones for each vertex. We use euclidean
    /// distance to determine the nearest bone
    void clusterize_euclidean(Cuda_utils::HA_int& h_nearest_bones,
                              Cuda_utils::HA_int& h_nearest_joint,
                              Cuda_utils::HA_int& nb_vert_by_bone);

    /// Update the attributes 'd_nearest_bone_in_device_mem' and
    /// 'd_nearest_joint_in_device_mem'
    void update_nearest_bone_joint_in_device_mem();

    /// Compute a radius for each bone, given the distance of each vertex to
    /// their closest bone
    void set_default_bones_radius();

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

    void compute_tangents(const Vec3_cu* vertices, Vec3_cu* tangents);

    ///  geometric deformation of vertices (SSD, dual quat ...)
    /// @param t : type of the deformation
    /// @param in : input vertices to deform
    /// @param out : output vertices deformed with 't' method
    void geometric_deformation(EAnimesh::Blending_type t,
                               const Cuda_utils::DA_Point_cu& d_in,
                               Vec3_cu* out);

    /// Interpolates between out_verts and ssd_position (in place)
    void ssd_lerp(Vec3_cu* out_verts);

    void fit_mesh(int nb_vert_to_fit,
                  int* d_vert_to_fit,
                  bool full_eval,
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

    /// Updating vbo nbo tbo with d_vert, d_normals and d_tangents.
    /// vertices with multiple texture coordinates are duplicated
    /// @param nb_vert size of the arrays. It's not necesarily the same as the
    /// buffer objects
    /// @param d_tangents array of tangents of size nb_vert if equal 0 this
    /// parameter is ignored
    /// @param tbo buffer object of tangents if equal 0 this parameter is
    /// ignored
    /// @warning buffer objects are to be registered in cuda context
    void update_opengl_buffers(int nb_vert,
                               const Vec3_cu* d_vert,
                               const Vec3_cu* d_normals,
                               const Vec3_cu* d_tangents,
                               GlBuffer_obj* vbo,
                               GlBuffer_obj* nbo,
                               GlBuffer_obj* tbo);

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

    /// Initialize attributes h_input_verts_per_bone and
    /// h_input_normals_per_bone which stores vertices and normals by nearest
    /// bones
    void init_verts_per_bone();

    void init_smooth_factors(Cuda_utils::DA_float& d_smooth_factors);

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    /// The mesh 'm' is deformed after each call of transform_vertices().
    /// deformation is computed from the initial position of the mesh stored in
    /// d_input_vertices. The mesh buffer objects attributes defines the animated
    /// mesh
    Mesh*      _mesh;
    Skeleton*  _skel;

    EAnimesh::Smooth_type mesh_smoothing;

    bool do_implicit_skinning;
    bool do_smooth_mesh;
    bool do_local_smoothing;
    bool do_interleave_fitting;
    bool do_update_potential;

    /// Smoothing strength after animation

    int smoothing_iter;
    int diffuse_smooth_weights_iter;
    float smooth_force_a; ///< must be between [0 1]
    float smooth_force_b; ///< must be between [0 1] only for humphrey smoothing
    float smooth_smear;   ///< between [-1 1]

    /// Mesh of voxels for display purpose
    Gl_mesh_quad* _vox_mesh;

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

    /// VBO of the Mesh in resting position
    GlBuffer_obj* _vbo_input_vert;
    /// NBO of the mesh in resting position
    GlBuffer_obj* _nbo_input_normal;

    std::vector<float> h_junction_radius;

    /// Stores in which state a vertex is when fitted into the implicit surface
    Cuda_utils::Device::Array<EAnimesh::Vert_state> d_vertices_state;
    /// Colors associated to the enum field EAnimesh::Vert_state
    Cuda_utils::Device::Array<float4> d_vertices_states_color;

    /// Initial normals
//    Cuda_utils::Device::Array<Vec3_cu> d_input_normals;

    /// Animated vertices in their final position.
    Cuda_utils::Device::Array<Point_cu>  d_output_vertices;
    /// final normals
    Cuda_utils::Device::Array<Vec3_cu> d_output_normals;
    Cuda_utils::Device::Array<Vec3_cu> d_output_tangents;

    /// Points of the mesh animated by ssd
    Cuda_utils::Device::Array<Point_cu>  d_ssd_vertices;
    /// Normals of the mesh animated by ssd
    Cuda_utils::Device::Array<Vec3_cu> d_ssd_normals;

    /// Gradient of the implicit surface at each vertices when animated
    Cuda_utils::Device::Array<Vec3_cu> d_gradient;

    /// triangle index in device mem. We don't use the mesh's vbos because
    /// there are different from the mesh's real topology as some of the vertices
    /// are duplicated for rendering because of textures
    Cuda_utils::Device::Array<int> d_input_tri;
    /// quad index in device mem. @see  d_input_tri
    Cuda_utils::Device::Array<int> d_input_quad;

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

    /// Map the ith packed vertex to the unpacked array form used for rendering
    /// d_packed_vert_map[packed_vert] = unpacked_vert
    /// @see Mesh
    Cuda_utils::Device::Array<Mesh::Packed_data> d_packed_vert_map;

    /// Vector representing the rotation axis of the nearest joint for each
    /// vertex. d_rot_axis[vert_id] == vec_rotation_axis
    Cuda_utils::Device::Array<Vec3_cu>  d_rot_axis;

    /// Vertices are moved based on a linear interpolation between ssd and
    /// implicit skinning this array map for each vertex index its interpolation
    /// weight : 1 is full ssd and 0 full implicit skinning
    Cuda_utils::Device::Array<float> d_ssd_interpolation_factor;

    // -------------------------------------------------------------------------
    /// @name SSD Weights
    // -------------------------------------------------------------------------

    /// List of joint numbers associated to a vertex. You must use d_jpv to
    /// access the list of joints associated to the ith vertex
    /// @see d_jpv
    Cuda_utils::Device::Array<int> d_joints;

    /// List of ssd weights associated to a vertex for animation. You must use
    /// d_jpv to access the list of ssd weights associated to the ith vertex
    /// @see d_jpv
    Cuda_utils::Device::Array<float> d_weights;

    /// Table of indirection which associates for the ith vertex its list of
    /// weights and joint IDs.
    /// For the ith vertex d_jpv gives for (ith*2) the starting index in
    /// d_joints and d_weights. The (ith*2+1) element gives the number of joint/
    /// weigths associated to the vertex in d_joints and d_weights arrays.
    Cuda_utils::Device::Array<int> d_jpv;

    /// SSD weigths on CPU
    std::vector<std::map<int, float> > h_weights;

    // -------------------------------------------------------------------------
    /// @name CLUSTER
    // -------------------------------------------------------------------------

    typedef Skeleton_env::DBone_id DBone_id;

    /// Mapping of mesh points with there nearest bone
    /// (i.e tab[vert_idx]=bone_idx)
    Cuda_utils::Host::  Array<Bone::Id>  h_vertices_nearest_bones;
    Cuda_utils::Device::Array<Bone::Id>  d_vertices_nearest_bones;
    Cuda_utils::Device::Array<DBone_id>  d_nearest_bone_in_device_mem;
    Cuda_utils::Device::Array<DBone_id>  d_nearest_joint_in_device_mem;

    /// Initial vertices in their "resting" position. sorted by nearest bone.
    /// h_input_vertices[nearest][ith_vert] = vert_coord
    std::vector< std::vector<Vec3_cu> > h_input_verts_per_bone;
    std::vector< std::vector<Vec3_cu> > h_input_normals_per_bone;

    /// h_input_vertices[nearest][ith_vert] = vert_id_in_mesh
    std::vector< std::vector<int> > h_verts_id_per_bone;

    /// Distance from the nearest bone to the mesh's vertices
    /// (distance is either euclidean or geodesic depending on the
    /// clusterisation)
    /// h_bone_dist[ver_id][bone_id] = dist bone to vert
    std::vector< double > h_nearest_bone_dist;

    /// nb_vertices_by_bones[bone_idx] = nb_vertices
    Cuda_utils::Host::Array<int>    nb_vertices_by_bones;

    /// Mapping of mesh points with there nearest joint
    /// (i.e tab[vert_idx] = joint_idx)
    Cuda_utils::Host::Array<int>   h_vertices_nearest_joint;
    Cuda_utils::Device::Array<int> d_vertices_nearest_joint;

    // END CLUSTER -------------------------------------------------------------

    /// Map old vertex index from Mesh class to the new Vertex index in
    /// Animesh. Because Animesh changes the order of vertex indices
    /// we need to keep track of that change. It is usefull to load from a file
    /// the ssd weight which are in the old order for instance.
    /// vmap_old_new[old_order_idx] == new_order_idx
    Cuda_utils::Host::Array<int> vmap_old_new;

    /// Map new vertex index from Animesh class to the old Vertex index in
    /// Mesh. Because Animesh changes the order of vertex indices
    /// we need to keep track of that change. It is usefull to export a file
    /// of ssd weights which usually must be stored as imported
    /// (in the old order).
    /// vmap_new_old[new_order_idx] == old_order_idx
    Cuda_utils::Host::Array<int> vmap_new_old;

    /// Vertices behind a joint when it flex
    Cuda_utils::Device::Array<bool> d_rear_verts;

    /// (joint idx in host mem)
    Cuda_utils::HA_Vec3_cu h_half_angles;
    Cuda_utils::DA_Vec3_cu d_half_angles;

    /// orthogonal vector at each joint considering the adjacents bones
    /// (joint idx in host mem)
    Cuda_utils::HA_Vec3_cu h_orthos;
    Cuda_utils::DA_Vec3_cu d_orthos;

    /// Array used to debug. Tells if a vertices needs to be fitted by following,
    /// the inverse direction it should take.
    /// d_flipp_propagation[vert_id] = if we flip the propagation dir of the fit
    Cuda_utils::Device::Array<bool> d_flip_propagation;

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
    Cuda_utils::Host::Array<int>        h_vert_to_fit_buff_2;
    /// @}
};
// END ANIMATEDMESH CLASS ======================================================

#endif // ANIMATED_MESH__

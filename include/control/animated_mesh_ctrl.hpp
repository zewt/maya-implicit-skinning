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
#ifndef ANIMATED_MESH_CTRL_HPP__
#define ANIMATED_MESH_CTRL_HPP__

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "vec3_cu.hpp"
#include "mesh.hpp"
#include "color.hpp"
#include "animesh_enum.hpp"
#include "transfo.hpp"
#include "bone_type.hpp"

// Forward definitions ---------------------------------------------------------
struct Animesh;
struct Skeleton;
namespace Loader { struct Abs_skeleton; }
class Camera;
// END Forward definitions -----------------------------------------------------

struct Samp_id{
    int bone_id; ///< bone identifier
    int samp_id; ///< sample identifier

    bool operator==(const Samp_id& id) const {
        return (id.bone_id == bone_id) && (id.samp_id == samp_id);
    }
};

/** @brief Settings Controler for the animated mesh
    This class provide a control interface for the animated mesh.
    Point selection and coloring are handle here.
*/

class Animated_mesh_ctrl {
private:
    struct Cap{
        std::vector<Vec3_cu> nodes;
        std::vector<Vec3_cu> n_nodes;
        bool enable;
    };

public:
    Animated_mesh_ctrl(Animesh* am);
    ~Animated_mesh_ctrl();

    void draw_rotation_axis();

    void enable_update_base_potential(bool state);

    void set_auto_precompute(bool s){ _auto_precompute = s; }
    void set_factor_siblings(bool s){ _factor_bones = s;    }
    void set_bone_type(int id, int bone_type);
    void set_draw_rot_axis(bool state);
    void set_display_points(bool state){ _display_points = state; }

    void update_base_potential();

    void update_clusters(int nb_voxels = 25);


    /// Set how the mesh is colored when edited
    //@{
    void color_type(EAnimesh::Color_type type);
    void color_uniform(float r, float g, float b, float a);
    void color_ssd_weights(int joint_id);
    //@}


    /// set a mesh point color (this is independant of the mesh color itself)
    void color_vertex(int vert, float r, float g, float b, float a);

    const Mesh* get_mesh() const;
    int get_nearest_bone(int vert_idx);
    bool is_point_displayed(){ return _display_points;     }

    //--------------------------------------------------------------------------
    /// @name Mesh deformation
    //--------------------------------------------------------------------------

    /// Compute the deformation according to the bone with the current
    /// parameters (dual quat or ssd or implicit skinning etc.)
    void deform_mesh();

    void set_do_smoothing(bool state);
    void set_smooth_factor(int i, float fact);
    void set_nb_iter_smooting(int nb_iter);

    void switch_do_smoothing();
    void switch_tune_direction();

    /// Set how the mesh is smoothed
    //@{
    void smooth_conservative();
    void smooth_laplacian();
    void smooth_tangential();
    void smooth_humphrey();
    //@}

    /// Do we smooth the whole mesh or just joints
    void set_local_smoothing(bool state);
    void set_smooth_force_a (float alpha);
    void set_smooth_force_b (float beta);
    void set_smooth_smear   (float val );

    void set_smoothing_weights_diffusion_iter(int nb_iter);

    int  get_nb_iter_smooth(){ return _nb_iter;       }
    int  get_blending_type (){ return _blending_type; }
    bool is_smooth_on      (){ return _do_smooth;     }

    /// invert the propagation direction (for the fitting) of the currently
    /// selected vertices
    void invert_propagation();

    /// Restore the default propagation direction of every vertices.
    void reset_invert_propagation();

    void do_ssd_skinning();
    void do_dual_quat_skinning();
    void set_implicit_skinning(bool s);

    //--------------------------------------------------------------------------
    /// @name SSD weights
    //--------------------------------------------------------------------------

    /// Set the weight of the ith vertex associated to the if joint,
    /// the value is clamped between [0, 1], and the value associated to the
    /// other joints are normalized.
    void set_ssd_weight(int id_vertex, int id_joint, float weight);
    /// SSD weights from the abstract representation of a parsed file.
    void set_ssd_weight(const Loader::Abs_skeleton& skel);
    
    //--------------------------------------------------------------------------
    /// @name Mesh_selection
    //--------------------------------------------------------------------------

    /// Compute the center of gravity of the points currently selected
    Vec3_cu cog_mesh_selection();

    int  get_nb_selected_points(){ return _selected_points.size(); }
    const std::vector<int>& get_selected_points(){ return _selected_points; }
    int* get_selected_points_device(){ return _d_selected_points; }

    //--------------------------------------------------------------------------
    /// @name File import/export
    //--------------------------------------------------------------------------

    void save_ism(const char* filename);
    void load_ism(const char* filename);
    void save_weights(const char* filename);
    void load_weights(const char* filename);

    //--------------------------------------------------------------------------
    /// @name HRBF Samples Handling
    //--------------------------------------------------------------------------

    /// Change the radius used to convert hrbf from global to compact
    void set_hrbf_radius(int bone_id, float rad);

    /// Given a 'bone_id' and its sample index delete the sample
    /// the operation updates rbf weights
    void delete_sample(int bone_id, int index);

    /// erase every samples of the bone
    void empty_samples(int bone_id);

    /// this method add a new sample given its position 'p' its normal 'n'
    /// @return the sample index
    int add_sample(int bone_id, const Vec3_cu& p, const Vec3_cu& n);

    void add_samples(int bone_id,
                     const std::vector<Vec3_cu>& p,
                     const std::vector<Vec3_cu>& n);

    /// @param bone_id bone identifier
    void choose_hrbf_samples_ad_hoc(int bone_id, float jmax, float pmax,  float minDist, float fold);

    void choose_hrbf_samples_poisson(int bone_id, float jmax, float pmax,  float minDist, int nb_samples, float fold);

    void choose_hrbf_samples_gael(int bone_id);

    void incr_junction_rad(int bone_id, float incr);

    /// Add joint cap for the ith bone ?
    void set_jcap(int bone_id, bool state);

    /// Add parent cap for the ith bone ?
    void set_pcap(int bone_id, bool state);

    /// Re-compute caps samples. Usefull when the skeleton joints change of position
    void update_caps(int bone_id, bool jcap, bool pcap);

    /// Transform the hrbf samples of the list bone_ids.
    /// This allows selection/display of samples even if the skeleton is moving
    /// @param bone_ids list of bone ids to be transformed. If the list is empty
    /// all samples are transformed
    void transform_samples(const std::vector<int>& bone_ids = std::vector<int>());


    /// draw the hrbf samples given the list of bone ids 'list'
    /// @param draw_normals if true draw the normals with GL_LINES
    /// @param rest_pose if true draw the samples in rest pose
    void draw_hrbf_points(const std::vector<int>& list,
                          bool draw_normals,
                          bool rest_pose);

    void draw_caps(const Cap& cap);

    //--------------------------------------------------------------------------
    /// @name HRBF Samples Selection
    //--------------------------------------------------------------------------
    const std::vector<Samp_id>& get_selected_samples(){
        return _selected_samples;
    }

    Vec3_cu get_sample_pos   (Samp_id id);
    Vec3_cu get_sample_normal(Samp_id id);

    /// Get sample position in animated position
    Vec3_cu get_sample_anim_pos(Samp_id id);
    /// Get sample normal in animated position
    Vec3_cu get_sample_anim_normal(Samp_id id);

    void transform_selected_samples(const Transfo& t);

    /// Compute the center of gravity of the currently selected samples
    Vec3_cu cog_sample_selection();


    // -------------------------------------------------------------------------
    /// @name Class Tools
    // -------------------------------------------------------------------------
private:
    /// add a point to the selection list and color it
    /// this method check for doubles before adding it
    void add_to_selection(int id);

    /// removes a point from the last selection do nothing if the point was not
    /// selected
    void remove_from_selection(int id);

    /// update _d_selected_points device array with _selected_points host
    void update_device_selection();

    /// Append the current caps and samples and update the 'bone_id' with them
    /// also allocates the '_anim_samples_list' for nodes and n_nodes
    void update_bone_samples(int bone_id);

    /// Replace all the samples from the skeleton in the attrib '_sample_list'
    void copy_bone_samples_to_list();


    /// Change the vector size of attr _sample_anim_list.nodes and .n_nodes
    void resize_samples_anim(int bone_id, int size);

    /// update buffer objects size when samples are added or deleted, it also
    /// updates gl buffer position and normals in rest position
    void update_gl_buffers_size(int size);

    /// update vbo storing rest position of points and normals
    void update_vbo_rest_pose(int size);

    /// How many samples for every bone
    int compute_nb_samples();

    /// @return number of samples in _samples_list untill we reach bone_id
    /// (excluded)
    int compute_offset(int bone_id);

    /// Convert all bones to the precomputed type. except for SSD bones and
    /// Already precomputed
    void precompute_all_bones();

    void transform_caps(int bone_id, const Transfo& tr);

    //--------------------------------------------------------------------------
    /// @name Tools file export
    //--------------------------------------------------------------------------

    /// Write the list of bone types (SSD HRBF etc.)
    void write_bone_types(std::ofstream& file);

    void write_samples(std::ofstream& file,
                       const std::vector<Vec3_cu>& nodes,
                       const std::vector<Vec3_cu>& n_nodes );

    /// Write the section related to the hrbf samples in '.ism' files
    void write_hrbf_env(std::ofstream& file);

    /// Write the section related to the hrbf caps in '.ism' files
    /// @param wether we write the joint cap list or parent cap list
    void write_hrbf_caps_env(std::ofstream& file, bool jcap);

    /// Write section of every ssd interpolation factors between ssd and is
    void write_ssd_is_lerp( std::ofstream& file );

    /// write section storing HRBF implicite primitives radius of the compact
    /// support
    void write_hrbf_radius( std::ofstream& file );

    //--------------------------------------------------------------------------
    /// @name Tools file import
    //--------------------------------------------------------------------------

    /// Read the section related to the hrbf samples in '.ism' files
    void read_hrbf_env(std::ifstream& file);

    void read_hrbf_env_weights( std::ifstream& file,
                                std::vector<std::vector<float4> >& bone_weights);

    /// Read the list of bone types (SSD HRBF etc.)
    void read_bone_types(std::ifstream& file,
                         std::vector<int>& bones_type);

    void read_samples(std::ifstream& file,
                      std::vector<Vec3_cu>& nodes,
                      std::vector<Vec3_cu>& n_nodes );

    void read_weights(std::ifstream& file,
                      std::vector<float4>& weights );

    /// read the section related to the hrbf caps in '.ism' files
    /// @param wether we read the joint cap list or parent cap list
    void read_hrbf_caps_env(std::ifstream& file, bool jcap);

    /// Read the section containing interpolation factor between ssd and is
    void read_ssd_is_lerp(std::ifstream& file);

    void read_hrbf_radius(std::ifstream& file,
                          std::vector<float>& radius_hrbf);

    //--------------------------------------------------------------------------
    /// @name Attributes
    //--------------------------------------------------------------------------

    bool _display;
    bool _display_points;
    bool _do_smooth;
    bool _draw_rot_axis;
    bool _auto_precompute; ///< Bones are always precomputed in grids
    bool _factor_bones;    ///< factor hrbf samples of siblings in a single bone
    int  _nb_iter;         ///< number of iterations for the mesh smoothing
    int  _blending_type;   ///< Animation type (SSD or dual Quat)

    std::vector<int>     _selected_points;  ///< List of selected mesh points
    std::vector<Samp_id> _selected_samples; ///< List of selected hrbf samples

    int* _d_selected_points;   ///< selected points on device mem

    struct Cap_list {
        Cap jcap;
        Cap pcap;
    };

    /// List of samples added to the bone's caps
    /// _bone_caps[bone_id] = two_caps
    std::vector<Cap_list> _bone_caps;
    std::vector<Cap_list> _bone_anim_caps;

    struct HSample_list{
        std::vector<Vec3_cu> nodes;
        std::vector<Vec3_cu> n_nodes;
    };

    /// List of samples used for each bone
    std::vector<HSample_list> _sample_list;

    /// List of samples in animated position (
    /// @warning animated position is only valid for currently selected bones
    std::vector<HSample_list> _sample_anim_list;

    /// Samples normals (GL_LINES) to be drawn
    GlBuffer_obj* _normals_bo;
    /// Samples normals (GL_LINES) in rest pose
    GlBuffer_obj* _rest_nbo;
    /// Samples in animated position
    GlBuffer_obj* _anim_bo;
    /// Samples in rest pose
    GlBuffer_obj* _rest_bo;
    /// Samples color
    GlBuffer_obj* _color_bo;

    Animesh* _animesh;
    Skeleton* _skel;
};

#endif // ANIMATED_MESH_CTRL_HPP__

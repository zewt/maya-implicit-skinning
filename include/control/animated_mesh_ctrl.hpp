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
#include "animesh_enum.hpp"
#include "transfo.hpp"
#include "bone_type.hpp"

// Forward definitions ---------------------------------------------------------
struct Animesh;
struct Skeleton;
namespace Loader { struct Abs_skeleton; }
// END Forward definitions -----------------------------------------------------

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
    bool is_smooth_on      (){ return _do_smooth;     }

    //--------------------------------------------------------------------------
    /// @name File import/export
    //--------------------------------------------------------------------------

    void save_ism(const char* filename);
    void load_ism(const char* filename);

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

    // Update bone_id using the specified HRBF sampler.  XXX: if this is kept, add an enum
    void update_hrbf_samples(int bone_id, int mode);
    void update_all_hrbf_samples(int mode);

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

private:
    /// Append the current caps and samples and update the 'bone_id' with them
    /// also allocates the '_anim_samples_list' for nodes and n_nodes
    void update_bone_samples(int bone_id);

    /// Change the vector size of attr _sample_anim_list.nodes and .n_nodes
    void resize_samples_anim(int bone_id, int size);

    /// How many samples for every bone
    int compute_nb_samples();

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

public:
    Animesh* _animesh;
    Skeleton* _skel;
};

#endif // ANIMATED_MESH_CTRL_HPP__

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
#include "sample_set.hpp"

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
public:
    Animated_mesh_ctrl(Mesh* mesh, Skeleton *skel);
    ~Animated_mesh_ctrl();

    void set_sampleset(const SampleSet &sample_set);

    void enable_update_base_potential(bool state);

    void set_auto_precompute(bool s){ _auto_precompute = s; }
    void update_base_potential();

    void update_clusters(int nb_voxels = 25);

    int get_nearest_bone(int vert_idx);

    //--------------------------------------------------------------------------
    /// @name Mesh deformation
    //--------------------------------------------------------------------------

    /// Compute the deformation according to the bone with the current
    /// parameters (dual quat or ssd or implicit skinning etc.)
    void deform_mesh();

    void set_do_smoothing(bool state);
    void set_smooth_factor(int i, float fact);
    void set_nb_iter_smooting(int nb_iter);

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

    // Get the current (possibly deformed) vertices, in their original order.
    void get_anim_vertices_aifo(std::vector<Point_cu>& anim_vert) const;

    // Copy the given vertices into the mesh.
    void copy_vertices(const std::vector<Vec3_cu> &vertices);

    // Get the default junction radius for each joint.  This can be used as a default _junction_radius
    // in SampleSet.
    void get_default_junction_radius(std::vector<float> &radius_per_joint) const;

    // Return the number of vertices in the mesh.  Calls to copy_vertices must have the
    // same number of vertices.
    int get_nb_vertices() const;

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

    void incr_junction_rad(int bone_id, float incr);

private:
    /// Append the current caps and samples and update the 'bone_id' with them
    /// also allocates the '_anim_samples_list' for nodes and n_nodes
    void update_bone_samples(int bone_id);

    /// Convert all bones to the precomputed type. except for SSD bones and
    /// Already precomputed
    void precompute_all_bones();

    //--------------------------------------------------------------------------
    /// @name Tools file export
    //--------------------------------------------------------------------------
#if !defined(NO_CUDA) // XXX: remove this stuff
    /// Write the list of bone types (SSD HRBF etc.)
    void write_bone_types(std::ofstream& file);

    /// Write the section related to the hrbf caps in '.ism' files
    /// @param wether we write the joint cap list or parent cap list
    void write_hrbf_caps_env(std::ofstream& file, bool jcap);

    /// write section storing HRBF implicite primitives radius of the compact
    /// support
    void write_hrbf_radius( std::ofstream& file );

    //--------------------------------------------------------------------------
    /// @name Tools file import
    //--------------------------------------------------------------------------

    void read_hrbf_env_weights( std::ifstream& file,
                                std::vector<std::vector<float4> >& bone_weights);

    /// Read the list of bone types (SSD HRBF etc.)
    void read_bone_types(std::ifstream& file,
                         std::vector<int>& bones_type);

    void read_weights(std::ifstream& file,
                      std::vector<float4>& weights );

    /// read the section related to the hrbf caps in '.ism' files
    /// @param wether we read the joint cap list or parent cap list
    void read_hrbf_caps_env(std::ifstream& file, bool jcap);

    void read_hrbf_radius(std::ifstream& file,
                          std::vector<float>& radius_hrbf);
#endif
    //--------------------------------------------------------------------------
    /// @name Attributes
    //--------------------------------------------------------------------------

    bool _auto_precompute; ///< Bones are always precomputed in grids
    bool _factor_bones;    ///< factor hrbf samples of siblings in a single bone
    int  _nb_iter;         ///< number of iterations for the mesh smoothing

    SampleSet _samples;

public:
    Animesh* _animesh;
    Skeleton* _skel;
};

#endif // ANIMATED_MESH_CTRL_HPP__

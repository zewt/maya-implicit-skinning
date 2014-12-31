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

#ifndef SAMPLE_SET_HPP
#define SAMPLE_SET_HPP

#include <vector>
#include <iostream>
#include <fstream>

#include "vec3_cu.hpp"
#include "transfo.hpp"

struct Animesh;
struct Skeleton;

namespace SampleSet
{
struct HSample_list {
    std::vector<Vec3_cu> nodes;
    std::vector<Vec3_cu> n_nodes;

    void clear()
    {
        nodes.clear();
        n_nodes.clear();
    }

    void append(const HSample_list &rhs)
    {
        nodes.insert(nodes.end(), rhs.nodes.begin(), rhs.nodes.end());
        n_nodes.insert(n_nodes.end(), rhs.n_nodes.begin(), rhs.n_nodes.end());
    }
};

struct Cap {
    Cap() { enable = false; }
    HSample_list samples;
    bool enable;
};

struct InputSample
{
    InputSample(): _junction_radius(0) { }

    Cap jcap;
    Cap pcap;

    /// List of samples
    HSample_list _sample_list;

    float _junction_radius;

    // Enable or disable joint and parent caps for the ith bone.  update_caps() must be called
    // after this is changed.
    void set_jcap(bool state);
    void set_pcap(bool state);

    // Add one or multiple samples to a bone, given a position and a normal.
    // Return the sample index.
    int add_sample(const Vec3_cu& p, const Vec3_cu& n);
    void add_samples(const std::vector<Vec3_cu>& p, const std::vector<Vec3_cu>& n);

    /// Given a 'bone_id' and its sample index, delete the sample.
    void delete_sample(int index);

    // Erase all samples from a bone.
    void clear();
};

struct SampleSet
{
    SampleSet(int nb_joints): _samples(nb_joints) { }

    std::vector<InputSample> _samples;

    // factor hrbf samples of siblings in a single bone
    bool _factor_bones;

    /// Transform the hrbf samples of the list bone_ids.
    /// This allows selection/display of samples even if the skeleton is moving
    /// @param bone_ids list of bone ids to be transformed. If the list is empty
    /// all samples are transformed
    void transform_samples(const std::vector<Transfo> &transfos, const std::vector<int>& bone_ids = std::vector<int>());

    /// Return the total number of samples for all bones.
    int compute_nb_samples() const;

    // Automatically choose samples for the given bone.
    void choose_hrbf_samples_ad_hoc(const Animesh &animesh, int bone_id, float jmax, float pmax,  float minDist, float fold);
    void choose_hrbf_samples_poisson(const Animesh &animesh, int bone_id, float jmax, float pmax,  float minDist, int nb_samples, float fold);

    // Re-compute caps samples. Useful when the skeleton joints change of position
    void update_caps(const Skeleton &skel, int bone_id, bool jcap, bool pcap);

    int get_all_bone_samples(const Skeleton &skel, int bone_id, HSample_list &out) const;

    /// Write the section related to the hrbf samples in '.ism' files
    void write_hrbf_env(std::ofstream& file) const;

    void write_samples(std::ofstream& file,
                       const std::vector<Vec3_cu>& nodes,
                       const std::vector<Vec3_cu>& n_nodes) const;

    /// Read the section related to the hrbf samples in '.ism' files
    void read_hrbf_env(std::ifstream& file);

    void read_samples(std::ifstream& file,
                      std::vector<Vec3_cu>& nodes,
                      std::vector<Vec3_cu>& n_nodes );

private:
    void transform_caps(int bone_id, const Transfo& tr);

    /// Compute caps at the tip of the bone to close the hrbf
    void compute_jcaps(const Skeleton &skel, int bone_id, HSample_list &out) const;

    /// Compute caps at the tip of the bone to close the hrbf
    /// @param use_parent_dir: add the cap following the parent direction and
    /// not the direction of 'bone_id'
    void compute_pcaps(const Skeleton &skel, int bone_id,
                                     bool use_parent_dir,
                                     HSample_list &out) const;
};

}

#endif

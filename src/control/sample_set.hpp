#ifndef SAMPLE_SET_HPP
#define SAMPLE_SET_HPP

#include <vector>
#include <map>
#include <iostream>
#include <fstream>

#include "vec3_cu.hpp"
#include "transfo.hpp"
#include "bone.hpp"
#include "transfo.hpp"

struct Skeleton;
class Mesh;
struct VertToBoneInfo;

namespace SampleSet
{
struct InputSample
{
    std::vector<Vec3_cu> nodes;
    std::vector<Vec3_cu> n_nodes;

    void clear();
    void append(const InputSample &rhs);

    // Add one or multiple samples to a bone, given a position and a normal.
    // Return the sample index.
    int add_sample(const Vec3_cu& p, const Vec3_cu& n);
    void add_samples(const std::vector<Vec3_cu>& p, const std::vector<Vec3_cu>& n);

    /// Given a 'bone_id' and its sample index, delete the sample.
    void delete_sample(int index);

    // Apply the given transformation to the samples.
    void transform(const Transfo &matrix);
};

struct SampleSetSettings
{
    SampleSetSettings():
        mode(Poisson),
        pcap(true),
        jcap(true),
        jmax(-0.2f),
        pmax(-0.2f),
        min_dist(0),
        nb_samples(50),
        fold(0)
    {
    }

    enum Mode
    {
        AdHoc,
        Poisson,
    };
    Mode mode;

    // factor hrbf samples of siblings in a single bone
    bool factor_bones;

    bool pcap;
    bool jcap;

    float jmax;
    float pmax;
    float min_dist;
    float fold;

    // Poisson only:
    int nb_samples;

    // The junction radius for each bone.  This is only used if pcap or jcap are enabled.
    std::map<Bone::Id,float> junction_radius;
};

struct SampleSet
{
    std::map<Bone::Id,InputSample> _samples;

    /// Transform the hrbf samples of the list bone_ids.
    /// This allows selection/display of samples even if the skeleton is moving
    /// @param bone_ids list of bone ids to be transformed. If the list is empty
    /// all samples are transformed
    void transform_samples(const std::vector<Transfo> &transfos, const std::vector<int>& bone_ids = std::vector<int>());

    // Automatically choose samples for the given bone.
    void choose_hrbf_samples(const Mesh *mesh, const Skeleton *skel, const VertToBoneInfo &vertToBoneInfo, const SampleSetSettings &settings, int bone_id);

    void get_all_bone_samples(Bone::Id bone_id, InputSample &out) const;

private:
    /// Compute caps at the tip of the bone to close the hrbf
    void compute_jcaps(const Skeleton &skel, const SampleSetSettings &settings, int bone_id, InputSample &out) const;

    /// Compute caps at the tip of the bone to close the hrbf
    void compute_pcaps(const Skeleton &skel, const SampleSetSettings &settings, int bone_id,
                                     InputSample &out) const;
};

}

#endif

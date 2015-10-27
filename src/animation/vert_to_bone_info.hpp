#ifndef VERT_TO_BONE_INFO_HPP
#define VERT_TO_BONE_INFO_HPP

#include "cuda_utils.hpp"
#include "bone.hpp"
#include <map>

struct Skeleton;
class Mesh;

struct VertToBoneInfo
{
    VertToBoneInfo(const Skeleton *skel, const Mesh *mesh, const std::vector< std::vector<Bone::Id> >&bones_per_vertex);

    /// h_input_vertices[nearest][ith_vert] = vert_id_in_mesh
    std::map<Bone::Id, std::vector<int> > h_verts_id_per_bone;

    /// Mapping of mesh points with there nearest bone
    /// (i.e tab[vert_idx]=bone_idx)
    std::vector<std::vector<Bone::Id> > bones_per_vertex;

    // Get the default junction radius for each joint.  This can be used as a default _junction_radius
    // in SampleSet.
    void get_default_junction_radius(const Skeleton *skel, const Mesh *mesh, std::map<Bone::Id,float> &nearest_rad) const;

    void get_default_hrbf_radius(const Skeleton *skel, const Mesh *mesh, std::map<Bone::Id,float> &out) const;
};

#endif

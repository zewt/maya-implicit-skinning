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

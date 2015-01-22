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

#include "vert_to_bone_info.hpp"
#include "mesh.hpp"
#include "skeleton.hpp"


VertToBoneInfo::VertToBoneInfo(const Skeleton *skel, const Mesh *mesh, const std::vector< std::vector<Bone::Id> > &bones_per_vertex):
    bones_per_vertex(bones_per_vertex)
{
    // Create an entry for each bone, even if the value is zero.
    for(Bone::Id bone_id: skel->get_bone_ids())
        h_verts_id_per_bone[bone_id];

    for(int vert_idx = 0; vert_idx < (int) bones_per_vertex.size(); ++vert_idx)
    {
        for(int bone_idx: bones_per_vertex[vert_idx])
            h_verts_id_per_bone[bone_idx].push_back(vert_idx);
    }
}

void VertToBoneInfo::get_default_junction_radius(const Skeleton *skel, const Mesh *mesh, std::map<Bone::Id,float> &nearest_rad) const
{
    const int nb_verts  = mesh->get_nb_vertices();

    const float inf = std::numeric_limits<float>::infinity();

    // Junction radius is nearest vertex distance
    for(Bone::Id bone_id: skel->get_bone_ids())
        nearest_rad[bone_id] = inf;

    for(int i = 0; i < nb_verts; i++)
    {
        const Point_cu vert = mesh->get_vertex(i).to_point();
        const std::vector<Bone::Id> &bone_ids = bones_per_vertex.at(i);
        for(Bone::Id bone_id: bone_ids) {
            float dist = skel->get_bone(bone_id)->dist_to(vert);

            nearest_rad[bone_id] = std::min(nearest_rad[bone_id], dist);
        }
    }

    for(Bone::Id i: skel->get_bone_ids())
    {
        if(nearest_rad[i] == inf)
            nearest_rad[i] = 1.f;
    }
}

void VertToBoneInfo::get_default_hrbf_radius(const Skeleton *skel, const Mesh *mesh, std::map<Bone::Id,float> &out) const
{
    const int nb_verts  = mesh->get_nb_vertices();

    std::map<Bone::Id, float> farthest_rad;
    for(int i = 0; i < nb_verts; i++)
    {
        const Point_cu vert = mesh->get_vertex(i).to_point();
        const std::vector<Bone::Id> &bone_ids = bones_per_vertex.at(i);
        for(Bone::Id bone_id: bone_ids) {
            float dist = skel->get_bone(bone_id)->dist_to( vert );

            farthest_rad[bone_id] = std::max(farthest_rad[bone_id], dist);
        }
    }

    // HRBF compact support radius is farthest vertex distance
    for(Bone::Id i: skel->get_bone_ids())
        out[i] = farthest_rad[i] == 0.f ? 1.f : farthest_rad[i];
}

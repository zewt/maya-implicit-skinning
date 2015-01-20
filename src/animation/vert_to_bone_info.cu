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


VertToBoneInfo::VertToBoneInfo(const Skeleton *skel, const Mesh *mesh):
    h_vertices_nearest_bones(mesh->get_nb_vertices())
{
    std::map<Bone::Id, std::vector<Vec3_cu> >& vertices = h_input_verts_per_bone;
    std::map<Bone::Id, std::vector<Vec3_cu> >& normals  = h_input_normals_per_bone;
    std::map<Bone::Id, std::vector<int>     >& vert_ids = h_verts_id_per_bone;
    vertices.clear();
    normals. clear();

    // Create an entry for each bone, even if the value is zero.
    for(Bone::Id bone_id: skel->get_bone_ids())
    {
        vertices[bone_id];
        normals[bone_id];
        vert_ids[bone_id];
    }

    clusterize_euclidean(skel, mesh, h_vertices_nearest_bones);

    for(int i = 0; i < mesh->get_nb_vertices(); i++)
    {
        int nearest = h_vertices_nearest_bones[i];

        if(mesh->is_disconnect(i))
            continue;

        const Vec3_cu vert = mesh->get_vertex(i);
        const Vec3_cu norm = mesh->get_normal(i);

        vertices[nearest].push_back( Vec3_cu(vert.x,  vert.y,  vert.z)              );
        normals [nearest].push_back( Vec3_cu(norm.x,  norm.y,  norm.z).normalized() );

        vert_ids[nearest].push_back( i );
    }
}

void VertToBoneInfo::clusterize_euclidean(const Skeleton *skel, const Mesh *mesh, std::vector<int> &vertices_nearest_bones)
{
    std::map<Bone::Id, int> nb_vert_by_bone;

    int n = mesh->get_nb_vertices();
    for(int i = 0; i < n ; i++)
    {
        float d0  = std::numeric_limits<float>::infinity();
        int   nd0 = 0;

        const Point_cu current_vertex = mesh->get_vertex(i).to_point();
        for(Bone::Id j: skel->get_bone_ids())
        {
            if(!skel->is_bone(j))
                continue;

            const Bone* b = skel->get_bone(j).get();

            // Compute nearest bone
            float dist2 = b->dist_sq_to(current_vertex);

            if(dist2 <= d0){
                d0  = dist2;
                nd0 = j;
            }
        }
        vertices_nearest_bones  [i] = nd0;
        nb_vert_by_bone[nd0]++;
    }

    for(auto &it: nb_vert_by_bone)
        printf("Bone %i has %i clustered vertices\n", it.first, it.second);
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
        const int bone_id = h_vertices_nearest_bones[i];
        const Point_cu vert = mesh -> get_vertex(i).to_point();
        float dist = skel->get_bone(bone_id)->dist_to( vert );

        nearest_rad [bone_id] = std::min(nearest_rad [bone_id], dist);
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
        const int bone_id = h_vertices_nearest_bones[i];
        const Point_cu vert = mesh -> get_vertex(i).to_point();
        float dist = skel->get_bone(bone_id)->dist_to( vert );

        farthest_rad[bone_id] = std::max(farthest_rad[bone_id], dist);
    }

    // HRBF compact support radius is farthest vertex distance
    for(Bone::Id i: skel->get_bone_ids())
        out[i] = farthest_rad[i] == 0.f ? 1.f : farthest_rad[i];
}

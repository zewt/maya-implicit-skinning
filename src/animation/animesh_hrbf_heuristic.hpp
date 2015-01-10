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

#ifndef ANIMESH_HRBF_HEURISTIC_HPP
#define ANIMESH_HRBF_HEURISTIC_HPP

struct Animesh;
class Mesh;
struct Skeleton;
#include "vert_to_bone_info.hpp"
#include "vec3_cu.hpp"
#include <vector>

/// @brief base class to find HRBF samples on the mesh given a bone
class HRBF_sampling {
public:
    HRBF_sampling(const Mesh *mesh, const Skeleton *skel);

    virtual ~HRBF_sampling(){ }

    /// Given the mesh and the defined bone _bone_id samples the mesh
    /// surface.
    virtual void sample(std::vector<Vec3_cu>& out_verts,
                        std::vector<Vec3_cu>& out_normals) const = 0;

    /// factor the samples if '_factor_siblings' is true. This returns
    /// The samples factored in the first bone for bones in the same level
    /// of the skeleton tree. The other children don't have samples
    void factor_samples(std::vector<int>& vert_ids,
                        std::vector<Vec3_cu>& verts,
                        std::vector<Vec3_cu>& normals) const;

    /// Eliminates samples too far from the bone using parameters "_jmax"
    /// and "_pmax"
    /// @warning reset skeleton before using this
    void clamp_samples(std::vector<int>& vert_ids,
                        std::vector<Vec3_cu>& verts,
                        std::vector<Vec3_cu>& normals) const;

    /// Bone to base the heuristic on
    int _bone_id;

    /// consider bones with multiple children has a single bone
    /// Samples will be added to the first child and other bone on the same
    /// level of the skeleton tree will be ignored.
    bool _factor_siblings;

    float _jmax; ///< percentage max dist from joint (range [-1 1])
    float _pmax; ///< percentage max dist from joint parent (range [-1 1])
    float _fold; ///< threshold scalar product dir projection/mesh normal

protected:
    const Mesh *mesh;
    const Skeleton *skel;
    const VertToBoneInfo vertToBoneInfo;
};

// -------------------------------------------------------------------------

/// @brief sampling the mesh using an adhoc heuristic
/// based on the mesh vertices
class Adhoc_sampling : public HRBF_sampling {
public:

    Adhoc_sampling(const Mesh *mesh, const Skeleton *skel):
        HRBF_sampling(mesh, skel),
        _mind(0.f)
    {}

    void sample(std::vector<Vec3_cu>& out_verts,
                std::vector<Vec3_cu>& out_normals) const;

    float _mind; ///< minimal distance between two samples
};

/// @brief sampling the mesh surface with a poisson disk strategy
class Poisson_disk_sampling : public HRBF_sampling {
public:

    Poisson_disk_sampling(const Mesh *mesh, const Skeleton *skel) :
        HRBF_sampling(mesh, skel),
        _mind(0.f),
        _nb_samples(0)
    {}

    void sample(std::vector<Vec3_cu>& out_verts,
                std::vector<Vec3_cu>& out_normals) const;

    ///< minimal distance between two samples (if == 0 e use _nb_samples)
    float _mind;
    int _nb_samples;
};

#endif


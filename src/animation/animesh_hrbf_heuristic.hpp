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
    HRBF_sampling(const Mesh *mesh, const Skeleton *skel, const VertToBoneInfo &vertToBoneInfo);

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

    float _jmax; // percentage max dist from joint (range [-1 1])
    float _pmax; // percentage max dist from joint parent (range [-1 1])
    float _fold; // threshold scalar product dir projection/mesh normal

protected:
    const Mesh *mesh;
    const Skeleton *skel;
    const VertToBoneInfo &vertToBoneInfo;
};

// -------------------------------------------------------------------------

/// @brief sampling the mesh using an adhoc heuristic
/// based on the mesh vertices
class Adhoc_sampling : public HRBF_sampling {
public:

    Adhoc_sampling(const Mesh *mesh, const Skeleton *skel, const VertToBoneInfo &vertToBoneInfo_):
        HRBF_sampling(mesh, skel, vertToBoneInfo_),
        _mind(0.f)
    {}

    void sample(std::vector<Vec3_cu>& out_verts,
                std::vector<Vec3_cu>& out_normals) const;

    float _mind; ///< minimal distance between two samples
};

/// @brief sampling the mesh surface with a poisson disk strategy
class Poisson_disk_sampling : public HRBF_sampling {
public:

    Poisson_disk_sampling(const Mesh *mesh, const Skeleton *skel, const VertToBoneInfo &vertToBoneInfo_) :
        HRBF_sampling(mesh, skel, vertToBoneInfo_),
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


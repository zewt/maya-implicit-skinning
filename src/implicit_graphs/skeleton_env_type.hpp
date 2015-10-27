#ifndef SKELETON_ENV_TYPE_HPP
#define SKELETON_ENV_TYPE_HPP

#include "tree_type.hpp"
#include "tree_cu_type.hpp"
#include "bone.hpp"
#include "blending_env_type.hpp"
#include "joint_type.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// Skeleton identifier for skeleton env
typedef int Skel_id;

/// Integer data linked to clusters
struct Cluster_cu {

    Cluster_cu() :
        nb_bone(-1), first_bone(-1),
        blend_type(EJoint::NONE), ctrl_id(-1)
    { }

    Cluster_cu(const Cluster& c) { *this = c; }

    Cluster_cu &operator=(const Cluster &c) {
        nb_bone    = c.nb_bone;
        first_bone = c.first_bone;
        blend_type = c.datas._blend_type;
        ctrl_id    = c.datas._ctrl_id;
        return *this;
    }

    /// Number of bones in the cluster
    int nb_bone;

    /// Id of the first bone in the cluster
    DBone_id first_bone;

    union {
        /// Blending type (clean union, max, ultimate ...)
        EJoint::Joint_t blend_type;
        /// number of cluster pairs in the blending list
        int nb_pairs;
    };
    union {
        /// Controller Id in Blending_env if needed by the operator
        Blending_env::Ctrl_id ctrl_id;
    };
};

/// Float data linked to to clusters
struct Cluster_data {
    float _bulge_strength;
};

/// Bone identifier in host memory layout for skeleton env
struct Hbone_id {
    Skel_id  _skel_id;
    Bone::Id _bone_id;

    Hbone_id() : _skel_id( -1 ), _bone_id( -1 ) { }
    Hbone_id(Skel_id skel_id, Bone::Id bone_id) :
        _skel_id(skel_id), _bone_id(bone_id)
    {  }

    /// Define order for std::map keys
    bool operator < (const Hbone_id& b) const {
        return b._skel_id == _skel_id ? (_bone_id < b._bone_id) :
                                        (_skel_id < b._skel_id);
    }
};

struct Offset {
    Offset() : list_data(-1), grid_data(-1) { }
    int list_data; ///< offset to acces data in blending list
    int grid_data; ///< offset to acces data in grid list
};

} // END SKELETON_ENV NAMESPACE ================================================

#endif // SKELETON_ENV_TYPE_HPP

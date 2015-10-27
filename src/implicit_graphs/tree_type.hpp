#ifndef TREE_TYPE_HPP
#define TREE_TYPE_HPP

#include "joint_type.hpp"
#include "blending_env_type.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

struct Joint_data {
    Joint_data() :
        _blend_type( EJoint::NONE ),
        _ctrl_id(-1),
        _bulge_strength(-1)
    {   }

    /// Blending type (clean union, max, ultimate ...)
    EJoint::Joint_t _blend_type;

    /// Controller Id in Blending_env if needed by the operator
    Blending_env::Ctrl_id _ctrl_id;

    /// Force of the bulge If blending is a bulge
    float _bulge_strength;
};

}// NAMESPACE END Skeleton_env  ================================================

#endif // TREE_TYPE_HPP

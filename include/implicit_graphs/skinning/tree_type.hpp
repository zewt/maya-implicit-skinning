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
#ifndef TREE_TYPE_HPP
#define TREE_TYPE_HPP

#include "joint_type.hpp"
#include "blending_env.hpp"

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

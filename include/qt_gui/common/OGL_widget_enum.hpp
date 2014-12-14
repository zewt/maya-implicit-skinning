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
#ifndef OGL_WIDGET_ENUM_HPP
#define OGL_WIDGET_ENUM_HPP

// =============================================================================
namespace EOGL_widget {
// =============================================================================

/// @name IO_t
/// @brief Input/output mode defines what the behavior and responses of
/// mouse/keyboard events
enum IO_t {
    RBF,       ///< Enables RBF sample editing
    DISABLE,   ///< Ignores all io events
    GRAPH,     ///< Editing the skeleton (add/remove/move joints)
    SKELETON,  ///< Skeleton manipulation (select/move)
    MESH_EDIT,  ///< Mesh editing (point selection)
    BLOB		///< Blob editing
};

/// @name Pivot_t
/// @brief Defines the center of rotation mode
enum Pivot_t {
    JOINT = 0,     ///< rotate around a joint
    BONE = 1,      ///< rotate around the midlle of a bone
    SELECTION = 2, ///< Rotate around the cog of selected elments
    USER = 3,      ///< Rotate around a point defined by the user
    FREE = 4       ///< no center of rotation
};

/// @name Select_t
/// @brief Defines the selection mode
enum Select_t {
    MOUSE,      ///< Use mouse cursor
    CIRCLE,     ///< Use circle area
    BOX,        ///< Use box area
    FREE_FORM   ///< Use free form
};

} // END OGL_widget ============================================================

#endif // OGL_WIDGET_ENUM_HPP

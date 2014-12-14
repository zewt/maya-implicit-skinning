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
#ifndef IO_SELECTION_ENUM_HPP__
#define IO_SELECTION_ENUM_HPP__

#include "color.hpp"

/// @name EIO_Selection
/// @brief enum used primarily with IO_selection class
/// @see IO_Selection Gizmo
// =============================================================================
namespace EIO_Selection {
// =============================================================================

/// Gizmo pivot point type
enum Pivot_t { MEDIAN,    ///< On median point of the current selection
               ACTIVE,    ///< Origin of the active object/data
               CURSOR_3D  ///< 3d cursor
               // TODO: individual objects centers
             };


/// Gizmo orientation
enum Dir_t {
    GLOBAL,   ///< aligned with world axis
    LOCAL,    ///< Object local space
    NORMAL,   ///< Object/data normal aligned
    VIEW      ///< Aligned with camera view
};

/// Transformation type mode
enum Transfo_t {
    TRANSLATION,
    ROTATION,
    SCALE,
    NONE         ///< no manipulation activated
};

/// Axis type (which axis or pair of axis are currently activated)
/// 'G' stands for global coordinates and 'L' for local coordinates
enum Axis_t {
    GX=0, LX=1,
    GY=2, LY=3,
    GZ=4, LZ=5,
    VIEW_PLANE = 6 ///< parralel to the image plane
};

/// @return true if 'a' is a global axis
static inline
bool is_global( Axis_t a ){ return (a == GX) || (a == GY) || (a == GZ); }

/// @return true if 'a' is a local axis
static inline
bool is_local ( Axis_t a ){ return (a == LX) || (a == LY) || (a == LZ); }

/// @return associated color to axis 'a'
static inline
Color get_axis_color( EIO_Selection::Axis_t a)
{
    Color c;
    switch(a){
    case LX: case GX:  c = Color(0.82f, 0.40f, 0.40f); break;
    case LY: case GY:  c = Color(0.40f, 0.82f, 0.40f); break;
    case LZ: case GZ:  c = Color(0.40f, 0.40f, 0.82f); break;
    default: break;
    }

    return c;
}

}// END EIO_SELCTION ===========================================================

#endif // IO_SELECTION_ENUM_HPP__

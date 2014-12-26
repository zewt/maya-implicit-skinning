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
#ifndef JOINT_TYPE_HPP__
#define JOINT_TYPE_HPP__

// TODO: to be moved in Skeleton_env

/** @namespace Joint_type
  @brief This namespace holds an enum field used to identify various joints types
*/
// =============================================================================
namespace EJoint{
// =============================================================================

/// @note the blending type list should be kept small. Too many blending
/// operators would slow down drastically the evaluation of the implicit skeleton
enum Joint_t {
    /// Gradient controlled operator with arc of circle profile.
    /// opening function is a diamond shape like.
    GC_ARC_CIRCLE_TWEAK = 0,
    /// clean union with the max function
    MAX,
    /// Gradient controlled bulge in contact
    BULGE,    
    NB_JOINT_T,
    NONE,
    /// Types with values higher than this enumerant
    /// will be interpreted as custom operators:
    /// Blending_env::Op_id id = enum_value - BEGIN_CUSTOM_OP_ID;
    BEGIN_CUSTOM_OP_ID
};

} // END JOINT_TYPE NAMESPACE ===================================================

#endif // JOINT_TYPE_HPP__

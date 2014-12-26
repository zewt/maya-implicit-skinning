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
#ifndef BONE_TYPE_HPP__
#define BONE_TYPE_HPP__

/** @namespace Bone_type
  @brief This namespace holds an enum field used to identify various bones types
*/
// =============================================================================
namespace EBone{
// =============================================================================

enum Bone_t {
    SSD,
    HRBF,
    PRECOMPUTED,
};

} // END BONE_TYPE NAMESPACE ===================================================

#endif // BONE_TYPE_HPP__

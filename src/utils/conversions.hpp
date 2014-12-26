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
#ifndef CONVERSIONS_HPP__
#define CONVERSIONS_HPP__

#include "point_cu.hpp"
#include "vec3_cu.hpp"


// =============================================================================
namespace Convs {
// =============================================================================

IF_CUDA_DEVICE_HOST
static inline Point_cu to_point(const Vec3_cu& v){
  return Point_cu(v.x, v.y, v.z);
}

/// Convert a Point_cu to a CUDA vector
IF_CUDA_DEVICE_HOST
static inline Vec3_cu to_vector(const Point_cu& v){
  return Vec3_cu(v.x, v.y, v.z);
}

}
// END Convert NAMESPACE =======================================================

#endif // CONVERSIONS_HPP__

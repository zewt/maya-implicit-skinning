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
#include <vector>
#include "maths/vec3_cu.hpp"

// =============================================================================
namespace Utils_sampling {
// =============================================================================

/// @param radius : minimal radius between every samples. If radius <= 0
/// a new radius is approximated given the targeted number of samples
/// 'nb_samples'
/// @param nb_samples : ignored if radius > 0 otherwise will try to match
/// and find  nb_samples by finding an appropriate radius.
/// @param verts : list of vertices
/// @param nors : list of normlas coresponding to each verts[].
/// @param tris : triangle indices in verts[] array.
/// @code
///     tri(v0;v1;v3) = { verts[ tris[ith_tri*3 + 0] ],
///                       verts[ tris[ith_tri*3 + 1] ],
///                       verts[ tris[ith_tri*3 + 2] ]   }
/// @endcode
/// @param samples_pos : resulting samples positions
/// @param samples_nors : resulting samples normals associated to samples_pos[]
/// @warning undefined behavior if (radius <= 0 && nb_samples == 0) == true
void poisson_disk(float radius,
                  int nb_samples,
                  const std::vector<Vec3_cu>& verts,
                  const std::vector<Vec3_cu>& nors,
                  const std::vector<int>& tris,
                  std::vector<Vec3_cu>& samples_pos,
                  std::vector<Vec3_cu>& samples_nor);

}// END UTILS_SAMPLING NAMESPACE ===============================================

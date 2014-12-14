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
#ifndef WEIGHTS_LOADER_HPP__
#define WEIGHTS_LOADER_HPP__

#include <vector>
#include <map>

/**
 * @namespace Weights_loader
 * @brief Load weights of influence for ssd skinning
 */
// =============================================================================
namespace Weights_loader {
// =============================================================================

/// @param nb_vert : number of vertices of the mesh the weights belongs to
/// @param weights : ssd weights per vertices per bones.
/// weights[ith_vert][bone_id] = ssd_weight
void load(const char* filename,
          int nb_vert,
          std::vector< std::map<int, float> >& weights);

}// END Weights_loader =========================================================

#endif // WEIGHTS_LOADER_HPP__

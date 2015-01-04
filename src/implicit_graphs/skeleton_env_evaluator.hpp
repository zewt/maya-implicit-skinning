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
#ifndef SKELETON_ENV_EVALUATOR_HPP__
#define SKELETON_ENV_EVALUATOR_HPP__

#include "skeleton_env.hpp"

/// @file skeleton_env_evaluator.hpp
/// @brief Holds methods to evaluate the implicit skeleton in the environment

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// @brief compute the potential of the whole skeleton
__device__
float compute_potential(Skel_id skel_id, const Point_cu& p, Vec3_cu& gf);

}// END Skeleton_env ===========================================================



#endif // SKELETON_ENV_EVALUATOR_HPP__

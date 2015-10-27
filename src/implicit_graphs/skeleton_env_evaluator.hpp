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

#ifndef TREE_TYPE_CU_HPP
#define TREE_TYPE_CU_HPP

#include "tree_type.hpp"
#include "identifier.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

DEFINE_IDENTIFIER(Cluster_id);
/// Bone identifier in device memory layout for skeleton env
DEFINE_IDENTIFIER(DBone_id  );

struct Cluster {
    int nb_bone;         ///< Number of bones in the cluster
    DBone_id first_bone; ///< Id of the first bone in the cluster
    Joint_data datas;    ///< datas linked to the cluster
};

}// NAMESPACE END Skeleton_env  ================================================

#endif // TREE_TYPE_CU_HPP

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

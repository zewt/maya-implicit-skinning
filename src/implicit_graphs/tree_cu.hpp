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
#ifndef TREE_CU_HPP
#define TREE_CU_HPP

#include <vector>
#include <map>
#include <list>

#include "tree.hpp"
#include "tree_cu_type.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// GPU representation of the tree
struct Tree_cu {
    Tree_cu(const Tree *tree);

    Cluster_id bone_to_cluster( DBone_id id) const {
        return _bone_to_cluster[id.id()];
    }

private:

    DBone_id compute_clusters(Bone::Id bid,
                              DBone_id acc,
                              std::vector<const Bone*>& bone_aranged,
                              std::vector<Cluster>& clusters,
                              std::vector<Cluster_id>& bone_to_cluster,
                              std::map<Bone::Id, DBone_id>& hidx_to_didx,
                              std::map<DBone_id, Bone::Id>& didx_to_hidx);

    /// blending type of a bone is defined by its parent.
    /// fill attributes '_blending_list' '_nb_pairs' '_nb_singletons'
    void compute_blending_list();

    /// @return wether its a pair or not
//    bool add_elt_to_blending_list(Cluster_id cid, const std::list<Cluster>& blending_list);

public:
    Bone::Id get_id_bone_aranged(int idx_bone_aranged) const {
        return _bone_aranged[ idx_bone_aranged ]->get_bone_id();
    }

    /// Add a cluster to the blending list.
    void add_cluster(Cluster_id cid, std::vector<Cluster> &out) const;

    /// Erase every elements from the blending list
    void clear_blending_list();

    /// list of Clusters to blend (GPU friendly memory layout)
    /// First part of the list is composed of pairs of cluster to be blended
    /// with a specific blending operator (there is 'nb_pairs' pairs)
    /// Last part of the list is composed with 'nb_singletons' singletons.
    /// Pairs and singletons are to be blended altogether with the max
    /// operator:
    /// max( pair_0, ..., pair_n )
    std::vector<Cluster> _blending_list;

    /// Tree we're building the GPU representation from
    const Tree *_tree;

    /// list of clusters _h_clusters[Cluster_id] = Cluster
    /// A cluster of bone can be blended with the operator of your choice
    std::vector<Cluster> _clusters;

    /// Bones list organized for device memory.
    /// Bones with same parents are contigus in memory.
    /// Index to acces this array must be a DBone_id.
    std::vector<const Bone*> _bone_aranged;

    /// Parents of bones in '_bone_aranged'
    /// _parents_arranged[DBone_id] = Dparent_bone
    std::vector<DBone_id> _parents_aranged;

    DBone_id hidx_to_didx(Bone::Id dbone_id) const { return _hidx_to_didx.at(dbone_id); }
    Bone::Id didx_to_hidx(DBone_id dbone_id) const { return _didx_to_hidx.at(dbone_id); }

private:
    /// Get the cluster associated to a bone
    /// _bone_to_cluster[DBone_id] = clus_id
    std::vector<Cluster_id> _bone_to_cluster;
private:

    /// host bone idx to device bone idx
    std::map<Bone::Id, DBone_id> _hidx_to_didx;

    /// device bone idx to host bone idx
    std::map<DBone_id, Bone::Id> _didx_to_hidx;
};


}// NAMESPACE END Skeleton_env  ================================================

#endif // TREE_CU_HPP

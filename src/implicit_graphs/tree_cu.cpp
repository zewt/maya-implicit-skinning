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
#include "tree_cu.hpp"

#include "ad_hoc_hand.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

Tree_cu::Tree_cu(Tree* tree) :
    _tree( tree ),
    _blending_list(*this)
{

    _clusters.       reserve( tree->bone_size() );
    _bone_aranged.   resize ( tree->bone_size() );
    _bone_to_cluster.resize ( tree->bone_size() );
    _parents_aranged.resize ( tree->bone_size() );

    int nb_bones = compute_clusters(tree->root(),
                                    DBone_id( 0 ),
                                    _bone_aranged,
                                    _clusters,
                                    _bone_to_cluster,
                                    _hidx_to_didx,
                                    _didx_to_hidx).id();

    assert((unsigned)nb_bones == tree->bone_size());
    assert((unsigned)compute_nb_cluster(tree->root()) == _clusters.size());

    // Build adjency for the new bone layout in '_bone_aranged'
    for(int i = 0; i < nb_bones; ++i)
    {
        Bone::Id hidx     = _bone_aranged[i]->get_bone_id();
        Bone::Id h_parent = tree->parent( hidx );
        if(h_parent == -1)
        {
            _parents_aranged[ i ] = DBone_id(-1);
            continue;
        }

        assert(_hidx_to_didx.find(h_parent) != _hidx_to_didx.end() );
        _parents_aranged[ i ] = _hidx_to_didx[ h_parent ];
    }

    compute_blending_list();
}

// -----------------------------------------------------------------------------

// Look up the bones in the tree:
//      if a bone is the first son of its parent then add a cluster
//      (root need to be treated as if there were a parent bone)
//      bones with same parents are concatenated in bone_aranged
DBone_id Tree_cu::compute_clusters(Bone::Id bid,
                                   DBone_id acc,
                                   std::vector<const Bone*>& bone_aranged,
                                   std::vector<Cluster>& clusters,
                                   std::vector<Cluster_id>& bone_to_cluster,
                                   std::map<Bone::Id, DBone_id>& hidx_to_didx,
                                   std::map<DBone_id, Bone::Id>& didx_to_hidx)
{
    int nb_psons = -1;
    const Bone::Id root_pson[] = { _tree->root() };
    const Bone::Id* psons = 0;

    int pt = _tree->parent( bid );
    if( bid == _tree->root() ) {
        psons = root_pson;
        nb_psons = 1;
    } else if(_tree->sons( pt ).size() > 0) {
        psons = &_tree->sons(pt)[0];
        nb_psons = (int)_tree->sons(pt).size();
    }

    if( bid == psons[0] )
    {
        // Create cluster and compute bone correspondances
        Cluster cs = {nb_psons, acc, Joint_data()};
        clusters.push_back( cs );

        for(int i = 0; i < nb_psons; i++)
        {
            hidx_to_didx[ psons[i] ] = acc;
            didx_to_hidx[ acc      ] = psons[i];

            bone_to_cluster[acc.id()] = Cluster_id((int) clusters.size() - 1); // cluster id
            bone_aranged   [acc.id()] = _tree->bone( psons[i] );
            acc++;
        }
    }

    const std::vector<int>& sons = _tree->sons( bid );
    for(unsigned i = 0; i < sons.size(); ++i)
    {
        Bone::Id idx = sons[i];
        acc = compute_clusters(idx,
                               acc,
                               bone_aranged,
                               clusters,
                               bone_to_cluster,
                               hidx_to_didx,
                               didx_to_hidx);
    }

    return acc;
}

void Tree_cu::compute_blending_list()
{
    _blending_list.clear();
    for(int cid = 0; cid < (int)_clusters.size(); ++cid)
        _blending_list.add_cluster( Cluster_id(0) + cid );
}

// -----------------------------------------------------------------------------

int Tree_cu::compute_nb_cluster(Bone::Id bid, int acc)
{
    const std::vector<int>& sons = _tree->sons( bid );
    if(sons.size() == 0) return acc;

    acc++;
    for(unsigned i = 0; i < sons.size(); ++i)
        acc = compute_nb_cluster(sons[i], acc);
    return acc;
}

// -----------------------------------------------------------------------------

void Tree_cu::BList::clear()
{
    _nb_pairs = 0;
    _nb_singletons = 0;
    _list.clear();
}

// -----------------------------------------------------------------------------

void Tree_cu::BList::  add_cluster(Cluster_id cid)
{
    Cluster cl = _tree_cu._clusters[cid.id()];
    DBone_id d_bone_id = cl.first_bone;
    Bone::Id h_bone_id = _tree_cu._bone_aranged[d_bone_id.id()]->get_bone_id();
    Bone::Id h_parent  = _tree_cu._tree->parent( h_bone_id );

#ifndef ENABLE_ADHOC_HAND
    //////////////////////
    // Blend with pairs //
    //////////////////////
    if( h_parent < 0 || _tree_cu._tree->data( h_parent )._blend_type == EJoint::MAX)
    {
        cl.datas = _tree_cu._tree->data(h_parent < 0 ? h_bone_id : h_parent);
        _list.push_back( cl );
        _nb_singletons++;
    }
    else
    {
        DBone_id d_parent = _tree_cu._parents_aranged[ d_bone_id.id() ];
        Cluster_id cid_parent = _tree_cu._bone_to_cluster[ d_parent.id() ];
        Cluster c0 = _tree_cu._clusters[ cid.id()        ];
        Cluster c1 = _tree_cu._clusters[ cid_parent.id() ];
        c0.datas = _tree_cu._tree->data(h_parent);
        c1.datas = _tree_cu._tree->data(h_parent);
        _list.push_front( c0 );
        _list.push_front( c1 );
        _nb_pairs++;
    }

#else
    ///////////////////////////
    // Blend with singletons //
    ///////////////////////////
    cl.datas = _tree_cu._tree->data(h_parent < 0 ? h_bone_id : h_parent);
    _list.push_back( cl );
    _nb_singletons++;
#endif
}

}// NAMESPACE END Skeleton_env  ================================================

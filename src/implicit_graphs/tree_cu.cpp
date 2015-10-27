#include "tree_cu.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

Tree_cu::Tree_cu(const Tree *tree) :
    _tree( tree )
{

    _clusters.       reserve( tree->bones().size() );
    _bone_aranged.   resize ( tree->bones().size() );
    _bone_to_cluster.resize ( tree->bones().size() );
    _parents_aranged.resize ( tree->bones().size() );

    int nb_bones = 0;
    for(const Bone *bone: tree->bones())
    {
        if(tree->parent(bone->get_bone_id()) != -1)
            continue;

        nb_bones = compute_clusters(bone->get_bone_id(),
                                        DBone_id(nb_bones),
                                        _bone_aranged,
                                        _clusters,
                                        _bone_to_cluster,
                                        _hidx_to_didx,
                                        _didx_to_hidx).id();
    }

    assert((unsigned)nb_bones == tree->bones().size());

    // Build adjency for the new bone layout in '_bone_aranged'
    for(int i = 0; i < nb_bones; ++i)
    {
        Bone::Id hidx     = _bone_aranged[i]->get_bone_id();
        Bone::Id h_parent = tree->parent( hidx );
        DBone_id parent_device_id = DBone_id(-1);
        if(h_parent != -1)
            parent_device_id = _hidx_to_didx.at(h_parent);
        _parents_aranged[i] = parent_device_id;
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
    const Bone::Id root_pson[] = { bid };
    const Bone::Id* psons = 0;

    int pt = _tree->parent( bid );
    if(pt == -1) {
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
        add_cluster(Cluster_id(cid), _blending_list);
}

void Tree_cu::clear_blending_list()
{
    _blending_list.clear();
}

// -----------------------------------------------------------------------------

void Tree_cu::add_cluster(Cluster_id cid, std::vector<Cluster> &out) const
{
    Cluster cl = _clusters[cid.id()];
    DBone_id d_bone_id = cl.first_bone;
    Bone::Id h_bone_id = _bone_aranged[d_bone_id.id()]->get_bone_id();
    Bone::Id h_parent  = _tree->parent( h_bone_id );

    //////////////////////
    // Blend with pairs //
    //////////////////////
    if( h_parent < 0 || _tree->data( h_parent )._blend_type == EJoint::MAX)
    {
        cl.datas = _tree->data(h_parent < 0 ? h_bone_id : h_parent);
        out.push_back( cl );

        Cluster empty;
        empty.nb_bone = 0;
        out.push_back( empty );
    }
    else
    {
        DBone_id d_parent = _parents_aranged[ d_bone_id.id() ];
        Cluster_id cid_parent = _bone_to_cluster[ d_parent.id() ];
        Cluster c0 = _clusters[ cid.id()        ];
        Cluster c1 = _clusters[ cid_parent.id() ];
        c0.datas = _tree->data(h_parent);
        c1.datas = _tree->data(h_parent);
        out.push_back( c0 );
        out.push_back( c1 );
    }
}

}// NAMESPACE END Skeleton_env  ================================================

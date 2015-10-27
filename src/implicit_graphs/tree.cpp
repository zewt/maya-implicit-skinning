#include "tree.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

Tree::Tree(const std::vector<const Bone*>& bones, const std::map<Bone::Id, Bone::Id>& parents):
    _parents(parents)
{
    for(const Bone *bone: bones)
    {
        Bone::Id bone_id = bone->get_bone_id();
        _bones[bone_id] = bone;
        int pt = _parents.at(bone_id);
        if(pt > -1) _sons[pt].push_back(bone_id);
    }

    // Make sure all bones have an entry in _sons and _datas, even if it's blank.
    for(const Bone *bone: bones) {
        _sons[bone->get_bone_id()];
        _datas[bone->get_bone_id()];
    }
}

// -----------------------------------------------------------------------------

/// Compute the axis aligned bounding box of the tree
BBox_cu Tree::bbox() const
{
    BBox_cu res;
    for(auto &it: _bones)
        res = res.bbox_union( it.second->get_bbox() );

    return res;
}


}// NAMESPACE END Skeleton_env  ================================================

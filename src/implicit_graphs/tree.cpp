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

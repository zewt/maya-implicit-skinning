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
#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include <map>
#include <set>
#include "bone.hpp"
#include "tree_type.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// CPU representation of the tree
struct Tree {
    Tree(const std::vector<const Bone*>& bones,
        const std::map<Bone::Id, Bone::Id>& parents);

    /// Compute the axis aligned bounding box of the tree
    BBox_cu bbox() const;

    // -------------------------------------------------------------------------
    /// @name Getter and Setters
    // -------------------------------------------------------------------------

    void set_bones(const std::vector<const Bone*>& bones) {
        for(const Bone *bone: bones)
            _bones.at(bone->get_bone_id()) = bone;
    }

    void set_joints_data(const std::map<Bone::Id, Joint_data>& datas){ _datas = datas; }

    bool is_leaf(Bone::Id bid) const { return _sons.at(bid).size() == 0; }

          std::vector<Bone::Id>& sons(Bone::Id hid)       { return _sons.at(hid); }
    const std::vector<Bone::Id>& sons(Bone::Id hid) const { return _sons.at(hid); }

    Joint_data& data(Bone::Id hid){ return _datas.at(hid); }
    const Joint_data& data(Bone::Id hid) const { return _datas.at(hid); }

    const Bone* bone(Bone::Id hid) const { return _bones.at(hid); }
    
    const std::set<const Bone*> bones() const {
        std::set<const Bone*> result;
        for(auto &it: _bones)
            result.insert(it.second);
        return result;
    }

    // int bone_size() const { return (int)_bones.size(); }

    Bone::Id parent(Bone::Id hid) const { return _parents.at(hid); }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

private:
    std::map<Bone::Id, std::vector<Bone::Id> > _sons;
    std::map<Bone::Id, Joint_data> _datas;
    std::map<Bone::Id, const Bone*> _bones;
    std::map<Bone::Id, Bone::Id> _parents;
};


}// NAMESPACE END Skeleton_env  ================================================

#endif // TREE_HPP

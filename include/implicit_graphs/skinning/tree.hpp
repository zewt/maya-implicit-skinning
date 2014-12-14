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
#include "bone.hpp"
#include "tree_type.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// CPU representation of the tree
struct Tree {

    Tree(int root,
         const std::vector<Bone*>& bones,
         const std::vector<int>& parents);

    /// Compute the axis aligned bounding box of the tree
    BBox_cu bbox() const;

    // -------------------------------------------------------------------------
    /// @name Getter and Setters
    // -------------------------------------------------------------------------

    void set_bones(const std::vector<Bone*>& bones){ _bones = bones; }

    void set_joints_data(const std::vector<Joint_data>& datas){ _datas = datas; }

    bool is_leaf(Bone::Id bid) const { return _sons[bid].size() == 0; }

    Bone::Id root() const { return _root; }

          std::vector<Bone::Id>& sons(Bone::Id hid)       { return _sons[hid]; }
    const std::vector<Bone::Id>& sons(Bone::Id hid) const { return _sons[hid]; }

    Joint_data& data(Bone::Id hid){ return _datas[hid]; }

          Bone* bone(Bone::Id hid)       { return _bones[hid]; }
    const Bone* bone(Bone::Id hid) const { return _bones[hid]; }

    int bone_size() const { return (int)_bones.size(); }

    Bone::Id parent(Bone::Id hid){ return _parents[hid]; }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

private:
    Bone::Id _root;
    std::vector< std::vector<Bone::Id> > _sons;
    std::vector<Joint_data> _datas;
    std::vector<Bone*> _bones;
    std::vector<Bone::Id> _parents;
};


}// NAMESPACE END Skeleton_env  ================================================

#endif // TREE_HPP

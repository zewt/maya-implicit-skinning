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

#ifndef BONE_SET_HPP
#define BONE_SET_HPP

#include <vector>
#include <map>
#include "loader_skel.hpp"
#include "bone.hpp"
class Bone;

struct BoneItem
{
    BoneItem(Bone *bone);
    ~BoneItem();

    void set_transforms(Transfo transfos);

    // XXX: std::unique_ptr?
    Bone *bone;

    // Bone in rest position.
    Bone_cu initial_position;
    Bone::Id parent;

    // The index in the initial list.  This is temporary, and assumes only one skeleton.
    int caller_idx;

private:
    BoneItem &operator=(BoneItem &rhs) { return *this; }
    BoneItem(const BoneItem &rhs) { }
};

class BoneSet
{
public:
    ~BoneSet() { unload(); }
    void load(const Loader::Abs_skeleton &skel);
    void unload();

    Bone *get_bone(Bone::Id bone_id) { return bones.at(bone_id).bone; }
    const Bone *get_bone(Bone::Id bone_id) const { return bones.at(bone_id).bone; }

    // Retrieve a bone according to the order it was added in the Abs_skeleton.  XXX: remove this
    Bone *get_bone_by_idx(int idx) { return const_cast<Bone*>(const_cast<const BoneSet *>(this)->get_bone_by_idx(idx)); }
    const Bone *get_bone_by_idx(int idx) const;

    std::vector<Bone*> all_bones() {
        std::vector<Bone*> result;
        for(auto &it: bones)
            result.push_back(it.second.bone);
        return result;
    }

    std::vector<const Bone*> all_bones() const {
        std::vector<const Bone*> result;
        for(auto &it: bones)
            result.push_back(it.second.bone);
        return result;
    }

    void set_transforms(const std::map<Bone::Id,Transfo> &transfos);

    std::map<Bone::Id, BoneItem> bones;
    std::map<Bone::Id, Bone::Id> parents;
};

#endif

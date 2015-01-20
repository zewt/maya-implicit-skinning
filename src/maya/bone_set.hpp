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
#include <memory>
#include "loader_skel.hpp"
#include "bone.hpp"
#include "sample_set.hpp"
class Bone;

struct BoneItem
{
    BoneItem(std::shared_ptr<Bone> bone);

    std::shared_ptr<Bone> bone;

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

    std::vector<std::shared_ptr<Bone> > all_bones() {
        std::vector<std::shared_ptr<Bone> > result;
        for(auto &it: bones)
            result.push_back(it.second.bone);
        return result;
    }

    std::map<Bone::Id, BoneItem> bones;
};

#endif

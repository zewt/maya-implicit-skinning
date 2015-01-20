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

#include "bone_set.hpp"
#include "bone.hpp"
#include "precomputed_prim.hpp"
#include "hrbf_env.hpp"
#include "sample_set.hpp"
#include "timer.hpp"

BoneItem::BoneItem(std::shared_ptr<Bone> bone_) {
    bone = bone_;
    parent = -1;
    caller_idx = -1;
}

void BoneSet::load(const Loader::Abs_skeleton &skel)
{
    unload();

    std::map<int,Bone::Id> loaderIdxToBoneId;
    std::map<Bone::Id,int> boneIdToLoaderIdx;

    // Create the Bones.
    for(int idx = 0; idx < (int) skel._bones.size(); idx++)
    {
        std::shared_ptr<Bone> bone(new Bone());

        BoneItem &item = bones.emplace(bone->get_bone_id(), bone).first->second;
        item.caller_idx = idx;

        loaderIdxToBoneId[idx] = item.bone->get_bone_id();
        boneIdToLoaderIdx[item.bone->get_bone_id()] = idx;
    }

    // Set up each BoneItem's parent.
    for(auto &it: bones)
    {
        BoneItem &item = it.second;
        Bone::Id bid = item.bone->get_bone_id();

        // Set up item.parent.
        int loader_idx = boneIdToLoaderIdx.at(bid);
        int loader_parent_idx = skel._parents[loader_idx];
        if(loader_parent_idx != -1)
            item.parent = loaderIdxToBoneId.at(loader_parent_idx);
    }

    // Set up each bone.
    for(auto &it: bones)
    {
        // Set up _bone.
        Bone::Id bone_id = it.first;
        BoneItem &item = it.second;
        int bone_loader_idx = boneIdToLoaderIdx.at(bone_id);

        Bone::Id parent_bone_id = item.parent;
        Vec3_cu org = Transfo::identity().get_translation();
        if(parent_bone_id != -1) {
            int parent_bone_loader_idx = boneIdToLoaderIdx.at(parent_bone_id);
            org = skel._bones[parent_bone_loader_idx].get_translation();
        }

        Vec3_cu end = skel._bones[bone_loader_idx].get_translation();
        Vec3_cu dir = end.to_point() - org.to_point();
        float length = dir.norm();

        Bone_cu initial_position = Bone_cu(org.to_point(), dir, length);
        item.bone->set_length(initial_position._length);
        item.bone->set_orientation(initial_position.org(), initial_position.dir());

        // If any bones lie on the same position as their parent, they'll have a zero length and
        // an undefined orientation.  Set them to a small length and a default orientation.
        if(item.bone->length() < 0.000001f)
        {
            item.bone->set_length(0.000001f);
            item.bone->_dir = Vec3_cu(1,0,0);
        }
    }
}

void BoneSet::unload()
{
    bones.clear();
}

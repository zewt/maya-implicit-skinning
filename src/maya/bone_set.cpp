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

BoneItem::BoneItem(Bone *bone_) {
    bone = bone_;
    parent = -1;
    caller_idx = -1;
}

BoneItem::~BoneItem() { delete bone; }

void BoneSet::load(const Loader::Abs_skeleton &skel)
{
    unload();

    std::map<int,Bone::Id> loaderIdxToBoneId;
    std::map<Bone::Id,int> boneIdToLoaderIdx;

    for(int idx = 0; idx < (int) skel._bones.size(); idx++)
    {
        Bone *bone = new Bone(1);

        BoneItem &item = bones.emplace(bone->get_bone_id(), bone).first->second;
        item.caller_idx = idx;

        loaderIdxToBoneId[idx] = item.bone->get_bone_id();
        boneIdToLoaderIdx[item.bone->get_bone_id()] = idx;
    }

    for(auto &it: bones)
    {
        BoneItem &item = it.second;
        Bone::Id bid = item.bone->get_bone_id();

        // XXX initial_position

        // Set up item.parent.
        int loader_idx = boneIdToLoaderIdx.at(bid);
        int loader_parent_idx = skel._parents[loader_idx];
        if(loader_parent_idx != -1)
            item.parent = loaderIdxToBoneId.at(loader_parent_idx);
    }










    std::map<Bone::Id, Transfo> _frames;
    for(auto &it: bones)
    {
        Bone::Id bid = it.first;
        int loader_idx = boneIdToLoaderIdx.at(bid);
        _frames[bid] = skel._bones[loader_idx];
    }

    for(auto &it: bones)
    {
        // Set up _bone.
        Bone::Id bid = it.first;
        BoneItem &item = it.second;

        Bone::Id parent_bone_id = item.parent;
        Vec3_cu org = Transfo::identity().get_translation();
        if(parent_bone_id != -1)
            org = _frames.at(parent_bone_id).get_translation();

        Vec3_cu end = _frames.at(bid).get_translation();
        Vec3_cu dir = end.to_point() - org.to_point();
        float length = dir.norm();

        item.initial_position = Bone_cu(org.to_point(), dir, length);
    }

    for(auto &it: bones)
    {
        BoneItem &item = it.second;

        // If any bones lie on the same position as their parent, they'll have a zero length and
        // an undefined orientation.  Set them to a small length, and the same orientation as their
        // parent.
        if(item.initial_position._length < 0.000001f)
        {
            item.initial_position._length = 0.000001f;

            if(item.parent != -1)
                item.initial_position._dir = bones.at(item.parent).initial_position._dir;
            else
                item.initial_position._dir = Vec3_cu(1,0,0);
        }

        item.bone->set_length( item.initial_position._length );
    }














    std::map<Bone::Id,Transfo> identity_transforms;
    for(auto &it: bones)
        identity_transforms[it.first] = Transfo::identity();
    set_transforms(identity_transforms);
}

void BoneSet::unload()
{
    bones.clear();
}

const Bone *BoneSet::get_bone_by_idx(int idx) const
{
    for(auto &it: bones)
        if(it.second.caller_idx == idx)
            return it.second.bone;

    assert(false);
    return NULL;
}

void BoneItem::set_transforms(Transfo bone_transform)
{
    Bone::Id i = bone->get_bone_id();

    // Check that we don't have a zero orientation.  It's an invalid value that will
    // trickle down through a bunch of other data as IND/INFs and eventually cause other
    // assertion failures, so flag it here to make it easier to debug.
    Bone_cu b = initial_position;
    assert(b.dir().norm_squared() > 0);

    bone->set_length( b.length() );
    bone->set_orientation(bone_transform * b.org(), bone_transform * b.dir());

    if(bone->get_type() == EBone::HRBF)
    {
        const int id = bone->get_hrbf().get_id();
        if( id > -1) HRBF_env::set_transfo(id, bone_transform);
    }
        
    if(bone->get_type() == EBone::PRECOMPUTED)
        bone->get_primitive().set_transform(bone_transform);
}

void BoneSet::load_sampleset_for_bone(const SampleSet::InputSample &sample_list, Bone::Id bone_id)
{
    Bone *bone = bones.at(bone_id).bone;

    if(sample_list.nodes.empty())
    {
        bone->set_enabled(false);
        return;
    }

    // Solve/compute compute HRBF weights
    Timer t;
    t.start();

    bone->set_enabled(true);
    bone->discard_precompute();
    bone->get_hrbf().init_coeffs(sample_list.nodes, sample_list.n_nodes);
    printf("update_bone_samples: Solved %i nodes in %f seconds\n", sample_list.nodes.size(), t.stop());

    // Make sure the current transforms are applied now that we've changed the bone.
    // XXX: If this is needed, Bone should probably do this internally.
    HRBF_env::apply_hrbf_transfos();
    Precomputed_prim::update_device_transformations();

    // XXX: It makes sense that we need this here, but if we don't call it we hang in a weird way.
    // Figure out why for diagnostics.
//    skel->update_bones_data();
}

void BoneSet::load_sampleset(const SampleSet::SampleSet &sample_set)
{
    for(auto &it: bones) {
        Bone::Id bone_id = it.first;
        SampleSet::InputSample sample_list;
        sample_set.get_all_bone_samples(bone_id, sample_list);
        load_sampleset_for_bone(sample_list, bone_id);
    }
}

void BoneSet::precompute_all_bones()
{
//    return;
    for(auto &it: bones)
    {
        Bone *bone = it.second.bone;
        if(bone->get_type() == EBone::HRBF)
            bone->precompute();
    }
}

// Note that after changing a bone's position, Skeleton.update_bones_data must be called
// for all skeletons that use it.
void BoneSet::set_transforms(const std::map<Bone::Id,Transfo> &transfos)
{
    // Put _anim_bones in the position specified by transfos.
    for(auto &it: bones)
    {
        BoneItem &item = it.second;

        // If this joint represents a bone, transform it by the parent joint's transform.
        Transfo bone_transform = Transfo::identity();
        if(item.parent != -1)
            bone_transform  = transfos.at(item.parent);

        item.set_transforms(bone_transform);
    }

    HRBF_env::apply_hrbf_transfos();
    Precomputed_prim::update_device_transformations();
}

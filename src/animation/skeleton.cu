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
#include "skeleton.hpp"

#include <iostream>
#include <fstream>

#include "std_utils.hpp"
#include "blending_env.hpp"
#include "skeleton_env.hpp"
#include "precomputed_prim.hpp"
#include "hrbf_kernels.hpp"
#include "hrbf_env.hpp"
#include "std_utils.hpp"
#include "cuda_utils.hpp"

namespace { __device__ void fix_debug() { } }

using namespace Cuda_utils;

void Skeleton::init_skel_env(bool single_bone)
{
    std::vector<const Bone*> bones;
    std::map<Bone::Id, Bone::Id> parents;
    for(auto &it: _joints) {
        bones.push_back(it.second._anim_bone.get());
        parents[it.first] = it.second._parent;
    }

    // If single_bone is true, tell Skeleton_env to create a grid with only one cell.
    // This effectively disables the grid optimization.  If we only have a single bone
    // then the grid is expensive to calculate and doesn't do us any good.  If single_bone
    // is false, use the default.  We do this here rather than with set_grid_res, so we
    // don't waste time calculating a 20x20x20 grid and then throwing it away.
    _skel_id = Skeleton_env::new_skel_instance(bones, parents, single_bone? 1:-1);

    update_bones_data();
    Skeleton_env::update_joints_data(_skel_id, get_joints_data());
}

Skeleton::Skeleton(std::vector<std::shared_ptr<const Bone> > bones, std::vector<Bone::Id> parents, bool single_bone)
{
    std::map<int,Bone::Id> loaderIdxToBoneId;
    std::map<Bone::Id,int> boneIdToLoaderIdx;

    // Create all of the SkeletonJoints, pointing at the Bone.
    for(int bid = 0; bid < (int) bones.size(); bid++)
    {
        std::shared_ptr<const Bone> bone = bones[bid];
        SkeletonJoint &joint = _joints[bone->get_bone_id()];
        joint._anim_bone = bone;

        loaderIdxToBoneId[bid] = bone->get_bone_id();
        boneIdToLoaderIdx[bone->get_bone_id()] = bid;
    }

    for(auto &it: _joints)
    {
        SkeletonJoint &joint = it.second;
        Skeleton_env::Joint_data d;
        d._blend_type     = EJoint::MAX;
        d._ctrl_id        = Blending_env::new_ctrl_instance();
        d._bulge_strength = 0.7f;
        joint._joint_data = d;

        joint._controller = IBL::Shape::caml();
        Blending_env::update_controller(d._ctrl_id, joint._controller);
    }

    for(auto &it: _joints)
    {
        Bone::Id bid = it.first;
        SkeletonJoint &joint = it.second;

        // Set up _children.
        int loader_idx = boneIdToLoaderIdx.at(bid);
        int loader_parent_idx = parents[loader_idx];
        joint._parent = loader_parent_idx == -1? -1: loaderIdxToBoneId.at(loader_parent_idx);
        if(joint._parent != -1)
            _joints.at(joint._parent)._children.push_back(bid);
    }

    // Create our Skeleton_env.
    init_skel_env(single_bone);
}

// -----------------------------------------------------------------------------

Skeleton::~Skeleton()
{
    for(auto &it: _joints) {
        auto joint = it.second;
        joint._children.clear();
        const int ctrl_id = joint._joint_data._ctrl_id;
        if( ctrl_id >= 0)
            Blending_env::delete_ctrl_instance(ctrl_id);
    }

    Skeleton_env::delete_skel_instance( _skel_id );
}

void Skeleton::set_joint_controller(int i,
                                    const IBL::Ctrl_setup& shape)
{
    _joints.at(i)._controller = shape;
    Blending_env::update_controller(_joints.at(i)._joint_data._ctrl_id, shape);
}

// -----------------------------------------------------------------------------

std::map<Bone::Id, Skeleton_env::Joint_data> Skeleton::get_joints_data() const
{
    std::map<Bone::Id, Skeleton_env::Joint_data> joints_data;
    for(auto it: _joints)
        joints_data[it.first] = it.second._joint_data;
    return joints_data;
}

void Skeleton::set_joint_blending(int i, EJoint::Joint_t type)
{
    _joints.at(i)._joint_data._blend_type = type;

    Skeleton_env::update_joints_data(_skel_id, get_joints_data());
}

// -----------------------------------------------------------------------------

void Skeleton::set_joint_bulge_mag(int i, float m)
{
    _joints.at(i)._joint_data._bulge_strength = std::min(std::max(m, 0.f), 1.f);
    Skeleton_env::update_joints_data(_skel_id, get_joints_data());
}

IBL::Ctrl_setup Skeleton::get_joint_controller(Bone::Id bone_id)
{
    return _joints.at(bone_id)._controller;
}

void Skeleton::update_bones_data()
{
    Skeleton_env::update_bones_data(_skel_id);
}

Skeleton_env::DBone_id Skeleton::get_bone_didx(Bone::Id i) const {
    return Skeleton_env::bone_hidx_to_didx(_skel_id, i);
}

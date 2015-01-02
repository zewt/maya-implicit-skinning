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
#include "precomputed_prim_env.hpp"
#include "hrbf_kernels.hpp"
#include "hrbf_env.hpp"
#include "conversions.hpp"
#include "std_utils.hpp"
#include "class_saver.hpp"
#include "loader_skel.hpp"
#include "globals.hpp"
#include "cuda_utils.hpp"

using namespace Cuda_utils;

const float default_bone_radius = 1.f;

void Skeleton::init_skel_env()
{
    std::vector<int> parents(_joints.size());
    std::vector<const Bone*> bones(_joints.size());
    for(int i = 0; i < (int) _joints.size(); ++i) {
        parents[i] = _joints[i]._parent;
        bones[i] = _joints[i]._anim_bone;
    }

    _skel_id = Skeleton_env::new_skel_instance(bones, parents);
    update_bones_pose();
    Skeleton_env::update_joints_data(_skel_id, get_joints_data());
    Skeleton_env::update_bones_data (_skel_id, bones);
}

Skeleton::Skeleton(const Loader::Abs_skeleton& skel)
{
    _joints.resize(skel._bones.size());

    for(int i = 0; i < (int) _joints.size(); i++)
    {
        Skeleton_env::Joint_data d;
        d._blend_type     = EJoint::MAX;
        d._ctrl_id        = Blending_env::new_ctrl_instance();
        d._bulge_strength = 0.7f;
        _joints[i]._joint_data = d;

        _joints[i]._controller = IBL::Shape::caml();
        Blending_env::update_controller(d._ctrl_id, _joints[i]._controller);

        _joints[i]._h_transfo = Transfo::identity();
    }

    std::vector<Transfo> _frames(skel._bones.size());
    for(int bid = 0; bid < (int) _joints.size(); bid++ )
        _frames[bid] = skel._bones[bid]._frame;

    for(int bid = 0; bid < (int) _joints.size(); bid++)
    {
        SkeletonJoint &joint = _joints[bid];

        joint._parent = skel._parents[bid];
        if(joint._parent != -1)
            _joints[joint._parent]._children.push_back(bid);

        int parent_bone_id = joint._parent;
        Vec3_cu org = Transfo::identity().get_translation();
        if(parent_bone_id != -1)
            org = _frames[parent_bone_id].get_translation();

        Vec3_cu end = _frames[bid].get_translation();

        Vec3_cu dir = end.to_point() - org.to_point();
        float length = dir.norm();

        joint._bone = Bone_cu(org.to_point(), dir, length, 0.f);
        // joint._bone = Bone_cu(org.to_point(), end.to_point(), 0.f);

    }

    // If any bones lie on the same position as their parent, they'll have a zero length and
    // an undefined orientation.  Set them to a small length, and the same orientation as their
    // parent.
    for(int bid = 0; bid < (int) _joints.size(); bid++)
    {
        SkeletonJoint &joint = _joints[bid];
        if(joint._bone._length >= 0.000001f)
            continue;
        joint._bone._length = 0.000001f;

        if(joint._parent != -1)
            joint._bone._dir = _joints[joint._parent]._bone._dir;
        else
            joint._bone._dir = Vec3_cu(1,0,0);
    }

    for(int bid = 0; bid < (int) _joints.size(); bid++)
    {
        SkeletonJoint &joint = _joints[bid];
        joint._anim_bone = new Bone_ssd();
        joint._anim_bone->set_length( joint._bone._length );
        joint._anim_bone->set_radius(default_bone_radius);
        joint._anim_bone->_bone_id = bid;
    }
    // must be called last
    init_skel_env();
}

// -----------------------------------------------------------------------------

Skeleton::~Skeleton()
{
    for(unsigned i = 0; i < _joints.size(); i++){
        _joints[i]._children.clear();
        delete _joints[i]._anim_bone;
        const int ctrl_id = _joints[i]._joint_data._ctrl_id;
        if( ctrl_id >= 0)
            Blending_env::delete_ctrl_instance(ctrl_id);
    }

    Skeleton_env::delete_skel_instance( _skel_id );
}

void Skeleton::set_joint_controller(Blending_env::Ctrl_id i,
                                    const IBL::Ctrl_setup& shape)
{
    assert( i >= 0);
    assert( i < (int) _joints.size());

    _joints[i]._controller = shape;
    Blending_env::update_controller(_joints[i]._joint_data._ctrl_id, shape);
}

// -----------------------------------------------------------------------------

std::vector<Skeleton_env::Joint_data> Skeleton::get_joints_data() const
{
    std::vector<Skeleton_env::Joint_data> joints_data(_joints.size());
    for(int i = 0; i < (int) _joints.size(); ++i)
        joints_data[i] = _joints[i]._joint_data;
    return joints_data;
}

void Skeleton::set_joint_blending(int i, EJoint::Joint_t type)
{
    assert( i >= 0);
    assert( i < (int) _joints.size());

    _joints[i]._joint_data._blend_type = type;

    Skeleton_env::update_joints_data(_skel_id, get_joints_data());
}

// -----------------------------------------------------------------------------

void Skeleton::set_joint_bulge_mag(int i, float m)
{
    assert( i >= 0);
    assert( i < (int) _joints.size());

    _joints[i]._joint_data._bulge_strength = std::min(std::max(m, 0.f), 1.f);
    Skeleton_env::update_joints_data(_skel_id, get_joints_data());
}

// -----------------------------------------------------------------------------

void Skeleton::set_bone(int i, Bone* b)
{
    assert(i < (int) _joints.size());
    assert(i >= 0);

    b->_bone_id = i;

    delete _joints[i]._anim_bone;
    _joints[i]._anim_bone = b;

    // Update _anim_bones (and HRBF/precomputed equivalents) for the new bone.
    // XXX: We're updating all bones, which means we're updating n^2 bones when we
    // convert all bones to precomputed.  Check if this is a performance issue.
    update_bones_pose();
//    // TODO: to be deleted update_hrbf_id_to_bone_id();
}

// -----------------------------------------------------------------------------

void Skeleton::set_bone_radius(int i, float radius)
{
    _joints[i]._anim_bone->set_radius(radius);
}

// -----------------------------------------------------------------------------

IBL::Ctrl_setup Skeleton::get_joint_controller(int i)
{
    assert( i >= 0);
    assert( i < (int) _joints.size());
    return _joints[i]._controller;
}

// -----------------------------------------------------------------------------

void Skeleton::set_bone_hrbf_radius(int i, float radius)
{
    _joints[i]._hrbf_radius = radius;

    if(bone_type(i) == EBone::HRBF)
    {
        ((Bone_hrbf*)_joints[i]._anim_bone)->set_hrbf_radius(radius);
    }
}

// -----------------------------------------------------------------------------

int Skeleton::get_hrbf_id(Bone::Id bone_id) const
{
    assert(bone_id >= 0);
    assert(bone_id < (int) _joints.size());
    if(bone_type(bone_id) == EBone::HRBF)
        return ((const Bone_hrbf*)_joints[bone_id]._anim_bone)->get_hrbf().get_id();
    else
        return -1;
}

// -----------------------------------------------------------------------------

float Skeleton::get_hrbf_radius(Bone::Id bone_id) const
{
    return _joints[bone_id]._hrbf_radius;
}

void Skeleton::transform_precomputed_prim()
{
    for( int i = 0; i < (int) _joints.size(); i++)
    {
        if(bone_type(i) != EBone::PRECOMPUTED)
            continue;

        Bone_precomputed *bone = (Bone_precomputed*) _joints[i]._anim_bone;
        Precomputed_prim &prim = bone->get_primitive();

        Precomputed_env::set_transform(prim.get_id(), get_bone_transform(i));
    }

    Precomputed_env::update_device_transformations();
}

void Skeleton::set_transforms(const std::vector<Transfo> &transfos)
{
    assert(transfos.size() == _joints.size());
    for(int i = 0; i < (int) _joints.size(); ++i)
        _joints[i]._h_transfo = transfos[i];
    update_bones_pose();
}

void Skeleton::update_bones_pose()
{
    // Put _anim_bones in the position specified by _h_transfo.
    for(unsigned i = 0; i < _joints.size(); i++)
    {
        SkeletonJoint &joint = _joints[i];

        // Check that we don't have a zero orientation.  It's an invalid value that will
        // trickle down through a bunch of other data as IND/INFs and eventually cause other
        // assertion failures, so flag it here to make it easier to debug.
        Bone_cu b = _joints[i]._bone;
        assert(b.dir().norm_squared() > 0);

        const Transfo bone_transform = get_bone_transform(i);
        _joints[i]._anim_bone->set_length( b.length() );
        _joints[i]._anim_bone->set_orientation(bone_transform * b.org(), bone_transform * b.dir());
    }

    // Transform HRBF bones:
    for (int i = 0; i < (int) _joints.size(); ++i)
    {
        const int id = get_hrbf_id(i);
        if( id > -1) HRBF_env::set_transfo(id, get_bone_transform(i));
    }

    HRBF_env::apply_hrbf_transfos();

    // Transform precomputed bones:
    transform_precomputed_prim();

    // In order to this call to take effect correctly it MUST be done after
    // transform_hrbf() and transform_precomputed_prim() otherwise bones
    // positions will not be updated correctly within the Skeleton_env.
    std::vector<const Bone*> bones(_joints.size());
    for(int i = 0; i < (int) _joints.size(); ++i)
        bones[i] = _joints[i]._anim_bone;

    Skeleton_env::update_bones_data(_skel_id, bones);
}

/*
  // TODO: to be deleted
void Skeleton::update_hrbf_id_to_bone_id()
{
    int res = 0;
    for(int i = 0; i < (int) _joints.size(); i++){
        if(bone_type(i) == EBone::HRBF){
            int hrbf_id = ((Bone_hrbf*)_anim_bones[i])->get_hrbf().get_id();
            res = std::max(hrbf_id , res);
        }
    }

    _hrbf_id_to_bone_id.clear();
    _hrbf_id_to_bone_id.resize(res+1);

    for(int i = 0; i < (int) _joints.size(); i++){
        if(bone_type(i) == EBone::HRBF){
            int hrbf_id = ((Bone_hrbf*)_anim_bones[i])->get_hrbf().get_id();
            _hrbf_id_to_bone_id[hrbf_id] = i;
        }
    }
}
*/

Skeleton_env::DBone_id Skeleton::get_bone_didx(Bone::Id i) const {
    return Skeleton_env::bone_hidx_to_didx(_skel_id, i);
}

const Transfo&  Skeleton::get_transfo(Bone::Id bone_id) const {
    assert(bone_id >= 0);
    assert(bone_id < (int) _joints.size());
    return _joints[bone_id]._h_transfo;
}

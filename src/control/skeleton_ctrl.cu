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
#include "skeleton_ctrl.hpp"

#include "animesh.hpp"
#include "globals.hpp"
#include "conversions.hpp"
#include "skeleton.hpp"

namespace { __device__ void fix_debug() { } }

Skeleton_ctrl::Skeleton_ctrl()
{
    skel = NULL;
}

Skeleton_ctrl::~Skeleton_ctrl()
{
    delete skel;
    skel = NULL;
}

void Skeleton_ctrl::load(const Loader::Abs_skeleton& abs_skel)
{
    delete skel;
    skel = new Skeleton(abs_skel);
}

void Skeleton_ctrl::set_transforms(const std::vector<Transfo> &transfos)
{
    skel->set_transforms(transfos);
}

// -----------------------------------------------------------------------------

bool Skeleton_ctrl::is_loaded() const { return skel != 0; }

int Skeleton_ctrl::get_hrbf_id(int bone_id)
{

    if( skel->bone_type(bone_id) != EBone::HRBF)
        return -1;

    Bone_hrbf* b = ((Bone_hrbf*)skel->get_bone(bone_id));
    return b->get_hrbf().get_id();
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_bone_id(int hrbf_id)
{
    for (int i = 0; i < skel->nb_joints(); ++i)
    {
        if( skel->bone_type(i) == EBone::HRBF)
        {
            Bone_hrbf* b = ((Bone_hrbf*)skel->get_bone(i));
            if(b->get_hrbf().get_id() == hrbf_id)
                return i;
        }
    }

    // hrbf id does not exists or hrbf are not even used
    return -1;
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_parent(int bone_id){
    return skel->parent(bone_id);
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_bone_type(int bone_id){
    return skel->bone_type(bone_id);
}

IBL::Ctrl_setup Skeleton_ctrl::get_joint_controller(int id_joint){
    int pt = skel->parent( id_joint );
    if( pt > -1)
        return skel->get_joint_controller(/*id_joint*/pt);
    else
        return IBL::Ctrl_setup();
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_controller(int id_joint, const IBL::Ctrl_setup& shape){
    int pt = skel->parent( id_joint );
    if( pt > -1)
        skel->set_joint_controller(/*id_joint*/pt, shape);
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_blending(int i, EJoint::Joint_t type){
    int pt = skel->parent( i );
    if(pt > -1)
        skel->set_joint_blending(pt, type);
}

// -----------------------------------------------------------------------------

EJoint::Joint_t Skeleton_ctrl::get_joint_blending(int id)
{
    int pt = skel->parent( id );
    if(pt > -1)
        return skel->joint_blending(pt);

    return EJoint::NONE;
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_bulge_mag(int i, float m){
    skel->set_joint_bulge_mag(i, m);
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_nb_joints(){
    return skel->nb_joints();
}

// -----------------------------------------------------------------------------

const std::vector<int>& Skeleton_ctrl::get_sons(int joint_id)
{
    return skel->get_sons( joint_id );
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::find_associated_bone(int hrbf_id)
{
    for(int i = 0; i < skel->nb_joints(); i++)
    {
        const Bone* b = skel->get_bone(i);
        if(b->get_type() == EBone::HRBF)
        {
            const HermiteRBF& hrbf = ((const Bone_hrbf*)b)->get_hrbf();
            if(hrbf.get_id() == hrbf_id)
                return i;
        }
    }
    return -1;
}

// -----------------------------------------------------------------------------

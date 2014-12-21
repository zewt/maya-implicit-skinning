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
#include "cuda_globals.hpp"
#include "globals.hpp"
#include "cuda_ctrl.hpp"
#include "conversions.hpp"
#include "skeleton.hpp"

// -----------------------------------------------------------------------------

void Skeleton_ctrl::load_pose(const std::string& filepath)
{
    g_skel->load_pose( filepath );
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::save_pose(const std::string& filepath)
{
    g_skel->save_pose( filepath );
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::root()
{
    return g_skel->root();
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::load( const Graph& g_graph )
{
    delete g_skel;
    g_skel = new Skeleton(g_graph, 0);

    reset_selection();
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::load(const Loader::Abs_skeleton& abs_skel)
{
    delete g_skel;
    g_skel = new Skeleton(abs_skel);

    reset_selection();
}

// -----------------------------------------------------------------------------

bool Skeleton_ctrl::is_loaded(){ return g_skel != 0; }

// -----------------------------------------------------------------------------

void Skeleton_ctrl::reset(){
    g_skel->reset();
}

// -----------------------------------------------------------------------------

Vec3_cu Skeleton_ctrl::joint_pos(int idx) {
    return g_skel->joint_pos(idx);
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::joint_anim_frame(int id_bone,
                              Vec3_cu& fx,
                              Vec3_cu& fy,
                              Vec3_cu& fz)
{
    Mat3_cu m = g_skel->joint_anim_frame(id_bone).get_mat3();
    fx = m.x();
    fy = m.y();
    fz = m.z();
}

// -----------------------------------------------------------------------------

Transfo Skeleton_ctrl::joint_anim_frame(int id_joint)
{
    return g_skel->joint_anim_frame( id_joint );
}

// -----------------------------------------------------------------------------

Transfo Skeleton_ctrl::bone_anim_frame(int id_bone)
{
    return g_skel->bone_anim_frame( id_bone );
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::reset_selection()
{
    _selected_joints.clear();
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::add_to_selection(int id)
{
    // Check for doubles
    bool state = false;
    for(unsigned int i=0; i<_selected_joints.size(); i++)
        state = state || (_selected_joints[i] == id);

    if(!state) _selected_joints.push_back(id);
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_hrbf_id(int bone_id)
{

    if( g_skel->bone_type(bone_id) == EBone::HRBF)
    {
        Bone_hrbf* b = ((Bone_hrbf*)g_skel->get_bone(bone_id));
        return b->get_hrbf().get_id();
    }
    else
        return -1;

}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_bone_id(int hrbf_id)
{
    for (int i = 0; i < g_skel->nb_joints(); ++i)
    {
        if( g_skel->bone_type(i) == EBone::HRBF)
        {
            Bone_hrbf* b = ((Bone_hrbf*)g_skel->get_bone(i));
            if(b->get_hrbf().get_id() == hrbf_id)
                return i;
        }
    }

    // hrbf id does not exists or hrbf are not even used
    return -1;
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_parent(int bone_id){
    return g_skel->parent(bone_id);
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_bone_type(int bone_id){
    return g_skel->bone_type(bone_id);
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_pose(Loader::Base_anim_eval* evaluator, int frame)
{
    if( g_skel == 0) return;

    std::vector<Transfo> trs( g_skel->nb_joints() );
    for(int i = 0; i < g_skel->nb_joints(); i++)
        trs[i] = evaluator->eval_lcl( i, frame );

    g_skel->_kinec->set_pose_lcl( trs );
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_pos(int joint_id, const Vec3_cu& pos)
{
   g_skel->set_joint_rest_pos(joint_id, Convs::to_point(pos));
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_offset_scale(const Vec3_cu& off, float scale)
{
    g_skel->set_offset_scale(off, scale);
}

// -----------------------------------------------------------------------------

IBL::Ctrl_setup Skeleton_ctrl::get_joint_controller(int id_joint){
    int pt = g_skel->parent( id_joint );
    if( pt > -1)
        return g_skel->get_joint_controller(/*id_joint*/pt);
    else
        return IBL::Ctrl_setup();
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_controller(int id_joint, const IBL::Ctrl_setup& shape){
    int pt = g_skel->parent( id_joint );
    if( pt > -1)
        g_skel->set_joint_controller(/*id_joint*/pt, shape);
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_blending(int i, EJoint::Joint_t type){
    int pt = g_skel->parent( i );
    if(pt > -1)
        g_skel->set_joint_blending(pt, type);
    //g_animesh->update_base_potential();
}

// -----------------------------------------------------------------------------

EJoint::Joint_t Skeleton_ctrl::get_joint_blending(int id)
{
    int pt = g_skel->parent( id );
    if(pt > -1)
        return g_skel->joint_blending(pt);

    return EJoint::NONE;
    //g_animesh->update_base_potential();
}

// -----------------------------------------------------------------------------

void Skeleton_ctrl::set_joint_bulge_mag(int i, float m){
    g_skel->set_joint_bulge_mag(i, m);
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::get_nb_joints(){
    return g_skel->nb_joints();
}

// -----------------------------------------------------------------------------

const std::vector<int>& Skeleton_ctrl::get_sons(int joint_id)
{
    return g_skel->get_sons( joint_id );
}

// -----------------------------------------------------------------------------

int Skeleton_ctrl::find_associated_bone(int hrbf_id)
{
    for(int i = 0; i < g_skel->nb_joints(); i++)
    {
        const Bone* b = g_skel->get_bone(i);
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

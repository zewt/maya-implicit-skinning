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

// -----------------------------------------------------------------------------

#ifndef PI
#define PI (3.14159265358979323846f)
#endif

using namespace Cuda_utils;

const float default_bone_radius = 1.f;

struct SkeletonImpl
{
    /// List of transformations associated to each bone in order to deform a mesh.
    /// A point will follow rigidly the ith bone movements if it is transformed
    /// by bone_transformations[parents[ith]].
    Cuda_utils::Host::PL_Array<Transfo> _h_transfos;
    /// same as h_transform but in device memory
    Cuda_utils::Device::Array<Transfo> _d_transfos;

    // TODO: this list might not be really needed as blending env already stores it
    /// shape of the controller associated to each joint
    /// for the gradient blending operators
    Cuda_utils::Host::Array<IBL::Ctrl_setup> _controllers;

    void init(int nb_joints)
    {
        _h_transfos.malloc(nb_joints);
        _d_transfos.malloc(nb_joints);
        _controllers.malloc(nb_joints);
    }

    typedef Cuda_utils::Host::PL_Array<Transfo> HPLA_tr;

    /// transform implicit surfaces computed with HRBF.
    /// @param global_transfos array of global transformations for each bone
    /// (device memory)
    void transform_hrbf(Skeleton *self, const Cuda_utils::Device::Array<Transfo>& d_global_transfos);

    /// transform implicit surfaces pre computed in 3D grids
    /// @param global_transfos array of global transformations for each bone
    void transform_precomputed_prim(Skeleton *self, const HPLA_tr& global_transfos);

    /// Tool function for update_vertices() method. This updates '_anim_bones'
    /// and '_anim_frames'
    void subupdate_vertices(Skeleton *self, int root, const HPLA_tr& global_transfos);
};

Transfo Skeleton::bone_anim_frame(int bone) const { return impl->_h_transfos[bone] * _bones[bone].get_frame(); }

const Transfo* Skeleton::d_transfos() const { return impl->_d_transfos.ptr(); }

void Skeleton::init(int nb_joints)
{
    impl->init(nb_joints);
    _nb_joints = nb_joints;
    _children.resize(nb_joints);
    _parents.resize(nb_joints);
    _frames.resize(nb_joints);
    _anim_frames.resize(nb_joints);
    _joints_data.resize(nb_joints);
    _anim_bones.resize(nb_joints);
    _bones.resize(nb_joints);
    _hrbf_radius.resize(nb_joints, 1.f);

    for(int i = 0; i < nb_joints; i++)
    {
        _anim_bones[i] = new Bone_ssd();

        Skeleton_env::Joint_data d;
        d._blend_type     = EJoint::MAX;
        d._ctrl_id        = Blending_env::new_ctrl_instance();
        d._bulge_strength = 0.7f;

        impl->_controllers[i] = IBL::Shape::caml();
        _joints_data[i] = d;
        Blending_env::update_controller(d._ctrl_id, impl->_controllers[i]);

        _anim_bones[i]->set_radius(default_bone_radius);
        _anim_bones[i]->_bone_id = i;

        impl->_h_transfos[i] = Transfo::identity();
    }

    _scale = 1.f;
    _offset = Vec3_cu::zero();
}

// -----------------------------------------------------------------------------

void Skeleton::init_skel_env()
{
    _skel_id = Skeleton_env::new_skel_instance(_root, _anim_bones, _parents);
    update_bones_pose();
    Skeleton_env::update_joints_data(_skel_id, _joints_data);
    Skeleton_env::update_bones_data (_skel_id, _anim_bones );
}

Skeleton::Skeleton(const Loader::Abs_skeleton& skel):
    _root(skel._root),
    impl(new SkeletonImpl())
{
    init( skel._bones.size() );

    for(unsigned i = 0; i < skel._bones.size(); i++ )
    {
        _frames    [i] = skel._bones[i]._frame;
        _parents   [i] = skel._parents[i];

        _anim_bones[i]->set_length( skel._bones[i]._length );

    }
    _children = skel._sons;

    fill_bones();
    // must be called last
    init_skel_env();
}

// -----------------------------------------------------------------------------

Skeleton::~Skeleton()
{
    for(unsigned i = 0; i < _anim_bones.size(); i++){
        _children[i].clear();
        delete _anim_bones[i];
        const int ctrl_id = _joints_data[i]._ctrl_id;
        if( ctrl_id >= 0)
            Blending_env::delete_ctrl_instance(ctrl_id);
    }

    Skeleton_env::delete_skel_instance( _skel_id );
}

void Skeleton::rec_to_string(int id, int depth, std::string& str)
{
    for (int i = 0; i < depth; ++i)
        str += "    ";

    str += "Bone: " + Std_utils::to_string(id) + " ";
    str += EBone::type_to_string( bone_type(id) ) + "\n";

    for(unsigned i = 0; i < _children[id].size(); ++i)
        rec_to_string( _children[id][i], depth+1, str);
}

// -----------------------------------------------------------------------------

std::string Skeleton::to_string()
{
    std::string str;
    rec_to_string(root(), 0, str);
    return str;
}

// -----------------------------------------------------------------------------

void Skeleton::set_joint_controller(Blending_env::Ctrl_id i,
                                    const IBL::Ctrl_setup& shape)
{
    assert( i >= 0);
    assert( i < _nb_joints);

    impl->_controllers[i] = shape;
    Blending_env::update_controller(_joints_data[i]._ctrl_id, shape);
}

// -----------------------------------------------------------------------------

void Skeleton::set_joint_blending(int i, EJoint::Joint_t type)
{
    assert( i >= 0);
    assert( i < _nb_joints);

    _joints_data[i]._blend_type = type;
    Skeleton_env::update_joints_data(_skel_id, _joints_data);
}

// -----------------------------------------------------------------------------

void Skeleton::set_joint_bulge_mag(int i, float m)
{
    assert( i >= 0);
    assert( i < _nb_joints);

    _joints_data[i]._bulge_strength = std::min(std::max(m, 0.f), 1.f);
    Skeleton_env::update_joints_data(_skel_id, _joints_data);
}

// -----------------------------------------------------------------------------

void Skeleton::set_bone(int i, Bone* b)
{
    assert(i < _nb_joints);
    assert(i >= 0);

    b->_bone_id = i;

    delete _anim_bones[i];
    _anim_bones[i] = b;

//    // TODO: to be deleted update_hrbf_id_to_bone_id();
}

// -----------------------------------------------------------------------------

void Skeleton::set_bone_radius(int i, float radius)
{
    _anim_bones[i]->set_radius(radius);
}

// -----------------------------------------------------------------------------

IBL::Ctrl_setup Skeleton::get_joint_controller(int i)
{
    assert( i >= 0);
    assert( i < _nb_joints);
    return impl->_controllers[i];
}

// -----------------------------------------------------------------------------

void Skeleton::set_bone_hrbf_radius(int i, float radius)
{
    _hrbf_radius[i] = radius;

    if(bone_type(i) == EBone::HRBF)
    {
        ((Bone_hrbf*)_anim_bones[i])->set_hrbf_radius(radius);
    }
}

// -----------------------------------------------------------------------------

int Skeleton::get_hrbf_id(Bone::Id bone_id) const
{
    assert(bone_id >= 0);
    assert(bone_id < _nb_joints);
    if(bone_type(bone_id) == EBone::HRBF)
        return ((const Bone_hrbf*)_anim_bones[bone_id])->get_hrbf().get_id();
    else
        return -1;
}

// -----------------------------------------------------------------------------

float Skeleton::get_hrbf_radius(Bone::Id bone_id) const
{
    return _hrbf_radius[bone_id];
}

Vec3_cu Skeleton::joint_pos(int joint) const {
    assert(joint >= 0        );
    assert(joint <  _nb_joints);
    return _anim_frames[joint].get_translation();
}

void SkeletonImpl::transform_hrbf(Skeleton *self, const Cuda_utils::Device::Array<Transfo>& d_global_transfos)
{
    for (int i = 0; i < self->nb_joints(); ++i)
    {
        const int id = self->get_hrbf_id(i);
        if( id > -1) HRBF_env::set_transfo(id, _h_transfos[i]);
    }

    HRBF_env::apply_hrbf_transfos();
}

// -----------------------------------------------------------------------------

void SkeletonImpl::transform_precomputed_prim(Skeleton *self, const HPLA_tr &global_transfos )
{
    // XXX: why does _nb_joints exist isntead of just _anim_bones.size()
    for( int i = 0; i < self->_nb_joints; i++)
    {
        if(self->bone_type(i) != EBone::PRECOMPUTED)
            continue;

        Bone_precomputed *bone = (Bone_precomputed*) self->_anim_bones[i];
        Precomputed_prim &prim = bone->get_primitive();
        Precomputed_env::set_transform(prim.get_id(), global_transfos[i]);
    }

    Precomputed_env::update_device_transformations();
}

void Skeleton::set_transforms(const std::vector<Transfo> &transfos)
{
    impl->_h_transfos.copy_from(transfos);
    update_bones_pose();
}

void Skeleton::update_bones_pose()
{
    // Update joints position in animated position and the associated
    // transformations
    impl->subupdate_vertices( this, _root, impl->_h_transfos );

    // Update joint positions in texture.
    impl->_d_transfos.copy_from( impl->_h_transfos );

    impl->transform_hrbf( this, impl->_d_transfos );
    impl->transform_precomputed_prim( this, impl->_h_transfos );

    // In order to this call to take effect correctly it MUST be done after
    // transform_hrbf() and transform_precomputed_prim() otherwise bones
    // positions will not be updated correctly within the Skeleton_env.
    Skeleton_env::update_bones_data(_skel_id, _anim_bones);
}

// -----------------------------------------------------------------------------

void SkeletonImpl::subupdate_vertices(Skeleton *self,  int root, const HPLA_tr& global_transfos)
{
    const Transfo tr = global_transfos[root];
    self->_anim_frames[root] = tr * self->_frames[root];

    Bone_cu b = self->_bones[root];
    self->_anim_bones[root]->set_length( b.length() );
    self->_anim_bones[root]->set_orientation(tr * b.org(), tr * b.dir());

    for(unsigned i = 0; i < self->_children[root].size(); i++)
        subupdate_vertices(self, self->_children[root][i], global_transfos);
}

// -----------------------------------------------------------------------------

/*
  // TODO: to be deleted
void Skeleton::update_hrbf_id_to_bone_id()
{
    int res = 0;
    for(int i = 0; i < _nb_joints; i++){
        if(bone_type(i) == EBone::HRBF){
            int hrbf_id = ((Bone_hrbf*)_anim_bones[i])->get_hrbf().get_id();
            res = std::max(hrbf_id , res);
        }
    }

    _hrbf_id_to_bone_id.clear();
    _hrbf_id_to_bone_id.resize(res+1);

    for(int i = 0; i < _nb_joints; i++){
        if(bone_type(i) == EBone::HRBF){
            int hrbf_id = ((Bone_hrbf*)_anim_bones[i])->get_hrbf().get_id();
            _hrbf_id_to_bone_id[hrbf_id] = i;
        }
    }
}
*/

void Skeleton::fill_bones()
{
    for(int bid = 0; bid < _nb_joints; bid++)
    {
        Vec3_cu org = _frames[bid].get_translation();
        Vec3_cu end = Vec3_cu::zero();
        int nb_sons = _children[bid].size();
        for(int s = 0; s < nb_sons; s++)
        {
            int sid = _children[bid][s];
            end += _frames[sid].get_translation();
        }
        end /= (float)nb_sons;

        if(nb_sons == 0 ){
            // We set a minimal length for the leaves
            _bones[bid] = Bone_cu(org.to_point(), _frames[bid].x(), 0.01f, 0.f);
            _anim_bones[bid]->set_length( 0.01f );
        }else{
            _bones[bid] = Bone_cu(org.to_point(), end.to_point(), 0.f);
            _anim_bones[bid]->set_length( (org-end).norm() );
        }

    }
}

// -----------------------------------------------------------------------------

Skeleton_env::DBone_id Skeleton::get_bone_didx(Bone::Id i) const {
    return Skeleton_env::bone_hidx_to_didx(_skel_id, i);
}

const Transfo&  Skeleton::get_transfo(Bone::Id bone_id) const {
    assert(bone_id >= 0);
    assert(bone_id < _nb_joints);
    return impl->_h_transfos[bone_id];
}

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
#include "kinematic.hpp"
#include "skeleton.hpp"

// -----------------------------------------------------------------------------

Kinematic::Kinematic(Skeleton& s) :
    _skel(s),
    _user_lcl(s.nb_joints()),
    _pose_lcl(s.nb_joints())
{
    for (int i = 0; i < _skel.nb_joints(); ++i) {
        _user_lcl[i] = Transfo::identity();
        _pose_lcl[i] = Transfo::identity();
    }
}

// -----------------------------------------------------------------------------


void Kinematic::set_pose_lcl( const std::vector<Transfo>& poses )
{
    _pose_lcl = poses;
    _skel.update_anim_pose();
}

// -----------------------------------------------------------------------------

void Kinematic::set_user_lcl_parent(int id_joint, const Transfo& tr)
{
    _user_lcl[id_joint] = tr;
    _skel.update_anim_pose();
}

// -----------------------------------------------------------------------------

void Kinematic::reset()
{
    for (int i = 0; i < _skel.nb_joints(); ++i) {
        _user_lcl[i] = Transfo::identity();
        _pose_lcl[i] = Transfo::identity();
    }
    _skel.update_anim_pose();
}

// -----------------------------------------------------------------------------

void Kinematic::compute_transfo_gl( Transfo* tr)
{
    rec_compute_tr( tr, _skel.root(), Transfo::identity() );
}

// -----------------------------------------------------------------------------

void Kinematic::rec_compute_tr(Transfo* transfos,
                              int root,
                              const Transfo& parent)
{
    // Joint frame in rest pose (global and local)
    int pid = _skel.parent(root) > -1  ? _skel.parent(root) : root;

    const Transfo f    = _skel.joint_frame    ( pid );
    //const Transfo finv = _skel.joint_frame_lcl( pid );

    // Global transfo of the pose for the current joint
    const Transfo pose = _skel.joint_frame( root ) * _pose_lcl[root] * _skel.joint_frame_lcl( root );

    // Frame of the parent joint with applied pose
    const Transfo ppose_frame = f * _pose_lcl[pid];

    // Global user transfo :

    // This will not work if we want both pose transformation and user transfo
    //const Transfo usr = f * _user_lcl[root] * finv;
    // The user transfo is based on the parent pose frame
    const Transfo usr = ppose_frame * _user_lcl[root] * ppose_frame.fast_invert();

    // Vertex deformation matrix with repercussion of the user transformations
    // throughout the skeleton's tree
    const Transfo tr = parent *  usr * pose;

    transfos[root] = tr;

    for(unsigned i = 0; i < _skel.get_sons( root ).size(); i++)
        rec_compute_tr(transfos, _skel.get_sons( root )[i], parent * usr);
}

// -----------------------------------------------------------------------------

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
#ifndef SKELETON_CTRL_HPP_
#define SKELETON_CTRL_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "vec3_cu.hpp"
#include "bone.hpp"
#include "bone_type.hpp"
#include "joint_type.hpp"
#include "blending_env_type.hpp"
#include "controller.hpp"
#include "transfo.hpp"
#include "loader_skel.hpp"

struct Skeleton;

class Skeleton_ctrl {
public:
    bool is_loaded() const;

    // -------------------------------------------------------------------------
    /// @name Setters
    // -------------------------------------------------------------------------

    void set_joint_blending(int i, EJoint::Joint_t type);
    void set_joint_bulge_mag(int i, float m);
    void set_joint_controller(int id_joint, const IBL::Ctrl_setup& shape);

    // -------------------------------------------------------------------------
    /// @name Getters
    // -------------------------------------------------------------------------

    /// @return the bone id associated to the hrbf -1 if the hrbf_id does not
    /// exists
    int get_bone_id(int hrbf_id);

    /// @return the parent id of the bone of index 'bone_id'
    int get_parent(int bone_id);

    int get_bone_type(int bone_id);

    EJoint::Joint_t get_joint_blending(int id);

    IBL::Ctrl_setup get_joint_controller(int id_joint);

    int get_nb_joints();

    const std::vector<int>& get_sons(int joint_id);

    std::unique_ptr<Skeleton> skel;
};


#endif // SKELETON_CTRL_HPP_

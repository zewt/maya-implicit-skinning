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
#include "vec3_cu.hpp"
#include "bone_type.hpp"
#include "joint_type.hpp"
#include "blending_env_type.hpp"
#include "controller.hpp"
#include "transfo.hpp"
#include "loader_skel.hpp"

struct Skeleton;

/**
    @brief
*/
class Skeleton_ctrl {
public:
    Skeleton_ctrl();
    ~Skeleton_ctrl();

    void load(const Loader::Abs_skeleton& abs_skel);

    bool is_loaded() const;

    /// @return the bone_id associated to the hrbf_id or -1
    int find_associated_bone(int hrbf_id);

    // -------------------------------------------------------------------------
    /// @name Setters
    // -------------------------------------------------------------------------

    void set_joint_blending(int i, EJoint::Joint_t type);

    void set_joint_bulge_mag(int i, float m);

    void set_joint_controller(int id_joint, const IBL::Ctrl_setup& shape);

    // -------------------------------------------------------------------------
    /// @name Getters
    // -------------------------------------------------------------------------

    /// @return identifier of the root bone
    int root();

    Vec3_cu joint_pos(int idx);

    /// Get local frame of the bone ( fx will  be oriented along the bone)
    /// no asumption can be done for fy and fz
    void joint_anim_frame(int id_bone,
                          Vec3_cu& fx,
                          Vec3_cu& fy,
                          Vec3_cu& fz);

    // Set the current joint transforms.
    void set_transforms(const std::vector<Transfo> &transfos);

    Transfo joint_anim_frame(int id_bone);

    Transfo bone_anim_frame(int id_bone);

    /// @return the hrbf indentifier in HRBF_Env or -1 if the designated bone
    /// is not a hrbf
    int get_hrbf_id(int bone_id);

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

    // -------------------------------------------------------------------------
    /// @name Selection
    // -------------------------------------------------------------------------

    const std::vector<int>& get_selection_set(){ return _selected_joints; }

    void reset_selection();

private:
    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------

    void add_to_selection(int id);

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    std::vector<int> _selected_joints; ///< set of selected skeleton joints

public: // XXX
    Skeleton *skel;
};


#endif // SKELETON_CTRL_HPP_

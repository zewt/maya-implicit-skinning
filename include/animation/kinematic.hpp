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
#ifndef KINEMATIC_HPP__
#define KINEMATIC_HPP__

#include <vector>
#include "transfo.hpp"

struct Skeleton;

/**
  @name Kinematic
  @brief Handling forward and inverse kinematic of the skeleton

  This class aims to handle the skeleton kinematic. While the skeleton
  only gives means to change each joint transformation this class
  provides more advanced functionnalities like passing on a user defined
  transformations to every children.



*/
class Kinematic {
public:
    Kinematic(Skeleton &s);

    /// Set the skeleton pose given a vector of local transformations at
    /// each joints. Usually we use this to animated a saved animation
    /// @note this updates the skeleton pose
    void set_pose_lcl( const std::vector<Transfo>& poses );

    /// Set fot the ith joint a local transformation expressed in the
    /// <b> parent </b> joint frame coordinate system
    /// @note this updates the skeleton pose
    void set_user_lcl_parent(int id_joint, const Transfo& tr);

    Transfo get_user_lcl_parent(int id_joint) const { return _user_lcl[id_joint]; }

    /// Given the current skeleton state the various local transformations
    /// at each joints we compute the global transformations used to deform
    /// the mesh's vertices. This method is called automatically by the skeleton
    /// to update it's state when a local transformation changes through.
    void compute_transfo_gl(Transfo *tr);

    void reset();

private:
    void rec_compute_tr(Transfo* transfos, int root, const Transfo& parent);

    Skeleton& _skel;

    /// Locale transformations of the skeleton's joints defined by the user.
    /// Applied on top of the pos transformations.
    /// These transformations are expressed in their <b> parent </b> joint frame
    std::vector<Transfo> _user_lcl;

    /// Locale transformations of the skeleton's joints. This is always applied
    /// first. It's Usually defined by the current animation frame pose
    std::vector<Transfo> _pose_lcl;
};

#endif // KINEMATIC_HPP__

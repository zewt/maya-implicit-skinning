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
#ifndef GL_SKELETON_HPP__
#define GL_SKELETON_HPP__

#include "glpick.hpp"
#include "camera.hpp"
#include "color.hpp"
#include <vector>

struct Skeleton;

/// @brief drawing skeleton with opengl
/// @see Skeleton
class GL_skeleton {
public:
    GL_skeleton(const Skeleton* skel);

    void draw_bone(int i, const Color& c, bool rest_pose, bool use_material, bool use_circle = false);

    int nb_joints() const;

    const Skeleton* _skel;
};


#endif // GL_SKELETON_HPP__

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
#ifndef GIZMO_SCALE2_HPP__
#define GIZMO_SCALE2_HPP__

#include "gizmo2.hpp"
#include "port_glew.h"
#include "vec3_cu.hpp"
#include "camera.hpp"

/**
  @name Gizmo_scale
  @brief 3D frame to provide GUI for object scaling.
  This class provide methods for scaling objects in the GUI.
  It memories the current scale and compute its new scale
  given the mouse position and axis/plane constraint. It also gives means to
  draw a custom oriented frame which is represented by arrows. Each arrow can be
  selected.

  @see Gizmo
*/

// TODO: to be implemented

class Gizmo_scale2 : public Gizmo2 {
public:

   Gizmo_scale2(): Gizmo2() { }

    /// draw the current selected point and frame. Frame is represented with
    /// arrows (green, red and blue) made of a cylinder and a cone for the tip.
    /// Frame is drawn at the position specified by '_frame.get_translation()'.
    /// X, Y and Z arrows orientations are defined by 'set_frame()'.
    void draw(const Camera& cam)
    {

    }

    /// select the x or y or z frame axis given a camera and a mouse position
    /// updates attributes (_axis or _plane) and _constraint
    /// @return true if one of the axis is selected
    bool select_constraint(const Camera& cam, int px, int py)
    {
        // slide_from();
        return false;
    }

    /// reset the selected constraint set by select_constraint()
    void reset_constraint()
    {

    }

    /// @brief scale object given the current constraint (plane or axis)
    /// If a constraint has been selected this will scale the selected object
    /// following the constraint and keeping the frame arrow
    /// as close as possible under the mouse position (px, py)
    /// @return the scaling made by the frame in world coordinates
    /// @see select_constraint()
    TRS slide(const Camera& cam, int px, int py)
    {
        return TRS();
    }

private:

};

#endif // GIZMO_SCALE2_HPP__

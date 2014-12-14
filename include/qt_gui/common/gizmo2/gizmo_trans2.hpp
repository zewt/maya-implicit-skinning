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
#ifndef GIZMO_TRANSLATION2_HPP__
#define GIZMO_TRANSLATION2_HPP__

#include "gizmo2.hpp"
#include "port_glew.h"
#include "vec3_cu.hpp"
#include "camera.hpp"

/**
  @name Gizmo_trans
  @brief 3D frame to provide GUI for points translations.
  This class provide methods for translating objects in the GUI.
  It memories the current position and compute its new position
  given the mouse position and axis/plane constraint. It also gives means to
  draw a custom oriented frame which is represented by arrows. Each arrow can be
  selected.

  @see Gizmo
*/

class Gizmo_trans2 : public Gizmo2 {
public:

    enum Constraint_t {AXIS, PLANE};
    enum Axis_t       {X , Y , Z , NOAXIS   };
    enum Plane_t      {XY, XZ, YZ, NOPLANE };

    Gizmo_trans2(): Gizmo2(),
        _size(10.f),
        _length(0.13f),
        _radius(0.30f/50.f),
        _ortho_factor(25.f),
        _selected_axis(NOAXIS),
        _selected_plane(NOPLANE),
        _axis(Vec3_cu::unit_x())
    { }

    /// draw the current selected point and frame. Frame is represented with
    /// arrows (green, red and blue) made of a cylinder and a cone for the tip.
    /// Frame is drawn at the position specified by '_frame.get_translation()'.
    /// X, Y and Z arrows orientations are defined by 'set_frame()'.
    void draw(const Camera& cam);

    /// select the x or y or z frame axis given a camera and a mouse position
    /// updates attributes (_axis or _plane) and _constraint
    /// @return true if one of the axis is selected
    bool select_constraint(const Camera& cam, int px, int py);

    /// reset the selected constraint set by select_constraint()
    void reset_constraint(){
        _selected_axis  = NOAXIS;
        _selected_plane = NOPLANE;
    }

    /// @brief slide point given the current constraint (plane or axis)
    /// If constraint has been selected this will move the selected point to
    /// its new position by following the constraint and keeping the frame
    /// as close as possible under the mouse position (px, py)
    /// @return the translation made by the frame in world coordinates
    /// (when clicked on)
    /// @see select_constraint()
    TRS slide(const Camera& cam, int px, int py);

    // TODO: slide along a plane parallel to the image plane

private:
    TRS slide_axis (Vec3_cu axis_dir, const Camera& cam, Vec2i_cu pix);
    TRS slide_plane(Vec3_cu normal  , const Camera& cam, int px, int py);

    Vec3_cu _color;       ///< point color
    float    _size;       ///< point size when drawing

    /// Current frame in which the point is moved
    //@{
    float _length;         ///< length of the arrows representing the frame
    float _radius;         ///< radius of the arrows representing the frame
    float _ortho_factor;
    //@}

    Constraint_t _constraint;     ///< current constraint (axis or plane movement)
    Axis_t       _selected_axis;  ///< current selected axis
    Plane_t      _selected_plane; ///< current selected plane

    Vec2i_cu     _pix_diff;       ///< pixels to substract when sliding
    Vec3_cu      _axis;           ///< current axis direction for movements
    Vec3_cu      _plane;          ///< current plane normal for movements
};

#endif // GIZMO_TRANSLATION2_HPP__

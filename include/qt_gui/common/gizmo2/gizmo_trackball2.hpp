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
#ifndef GIZMO_TRACKBALL2_HPP__
#define GIZMO_TRACKBALL2_HPP__

#include "gizmo2.hpp"
#include "vec3_cu.hpp"
#include "transfo.hpp"
#include "camera.hpp"
#include "glpick.hpp"

/**
  @class Gizmo_trackball
  @brief Specialization of the gizmo to handle trackballs movements

  A trackball enables the user to grab anywhere on the screen a virtual ball
  and move it according to the dragged point. usually more intuitive to achieve
  complex rotations.

  @see Gizmo
*/
class Gizmo_trackball2 : public Gizmo2 {
public:

    enum Axis_t {BALL,
                 NOAXIS
                };

    Gizmo_trackball2();

    void draw(const Camera& cam);

    /// @note always returns true (except when show == false).
    /// Call is needed for the gizmo to compute the rotation
    bool select_constraint(const Camera& cam, int px, int py);

    void reset_constraint(){ _selected_axis = NOAXIS; }

    TRS slide(const Camera& cam, int px, int py);

private:
    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------
    TRS trackball_rotation(const Camera& cam, int px, int py);

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    float _rad_sphere;    ///< Radius of the sphere representing the trackball
    float _rad_handles;   ///< Radius of the tore reprensenting the handles
    float _ortho_factor;

    /// current selected axis
    Axis_t _selected_axis;

    /// World coordinates of the picked point
    Vec3_cu _picked_pos;
};

#endif // GIZMO_TRACKBALL2_HPP__

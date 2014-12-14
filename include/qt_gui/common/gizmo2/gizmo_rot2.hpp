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
#ifndef GIZMO2_ROT_HPP__
#define GIZMO2_ROT_HPP__

#include "gizmo2.hpp"
#include "vec3_cu.hpp"
#include "transfo.hpp"
#include "camera.hpp"
#include "glpick.hpp"
#include "color.hpp"

class Gizmo_rot2 : public Gizmo2 {
public:

    enum Axis_t {X = 0,    ///< Rotation around X axis
                 Y,        ///< Rotation around Y axis
                 Z,        ///< Rotation around X axis
                 CAM,      ///< Rotation around the camera view dir axis
                 NOAXIS    ///< no axis constraint selected
               };

    Gizmo_rot2();

    void draw(const Camera& cam);

    bool select_constraint(const Camera& cam, int px, int py);

    void reset_constraint(){ _selected_axis = NOAXIS; }

    TRS slide(const Camera& cam, int px, int py);

private:
    // =========================================================================
    /// @name Tools
    // =========================================================================
    TRS axis_rotation(const Camera& cam, int px, int py);

    /// Draw the tangent of the selected circle
    void draw_tangent(const Vec3_cu& cam_dir, float dist_cam);

    /// Draw the arc circle corresponding to the axis 'axis'
    void draw_arc_circle(Axis_t axis,
                         const Vec3_cu& a,
                         const Vec3_cu& cam_dir,
                         float scale,
                         const Color& cl);

    void draw_circle(Axis_t axis,
                     const Vec3_cu& cam_dir,
                     float scale,
                     const Color& cl);

    // =========================================================================
    /// @name Attributes
    // =========================================================================

    float _rad_sphere;    ///< Radius of the sphere representing the trackball
    float _rad_handles;   ///< Radius of the tore reprensenting the handles
    float _ortho_factor;

    Vec3_cu _axis;         ///< current axis direction for movements
    Axis_t _selected_axis; ///< current selected axis

    /// pixel clicked when during constraint selection
    //Vec2i_cu _clicked;

    /// World coordinates of the picked point
    Vec3_cu _picked_pos;
    /// Tangent of the selected circle
    Vec3_cu _picked_tangent;

    GLPick _pick;
};

#endif // GIZMO2_ROT_HPP__

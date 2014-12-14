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
#include "gizmo_trackball.hpp"

#include "glsave.hpp"
#include "globals.hpp"
#include "glu_utils.hpp"
#include "conversions.hpp"
#include "trackball.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// -----------------------------------------------------------------------------

Gizmo_trackball::Gizmo_trackball() : Gizmo(),
    _rad_sphere(6.5f),
    _rad_handles(1.0f),
    _ortho_factor(20.f),
    _selected_axis(NOAXIS)
{
    _frame = Transfo::identity();
}

// -----------------------------------------------------------------------------

void Gizmo_trackball::draw(const Camera& cam)
{
    if(!_show) return;

    GLEnabledSave save_light(GL_LIGHTING  , true, false);
    GLEnabledSave save_textu(GL_TEXTURE_2D, true, false);
    GLEnabledSave save_depth(GL_DEPTH_TEST, true, true);

    Vec3_cu org = _frame.get_translation();
    const float dist_cam =  !cam.is_ortho() ? (org-cam.get_pos()).norm() : _ortho_factor;
    float s = dist_cam / 50.f;

    GLLineWidthSave save_line_width;

    glLineWidth( _rad_handles );

    glPushMatrix();
    {
        glMultMatrixf( _frame.transpose().m );
        glScalef(s, s, s);
        glScalef(_rad_sphere, _rad_sphere, _rad_sphere);

        glColor4f(1.f, 1.f, 1.f, 1.f);

        // X axis
        glPushMatrix();
        glRotatef(90.f, 0.f, 1.f, 0.f);
        g_primitive_printer.draw( g_circle_vbo );
        glPopMatrix();

        // Y axis
        glPushMatrix();
        glRotatef(90.f, 1.f, 0.f, 0.f);
        g_primitive_printer.draw( g_circle_vbo );
        glPopMatrix();

        // Z Axis
        g_primitive_printer.draw( g_circle_vbo );
    }
    glPopMatrix();
}

// -----------------------------------------------------------------------------

bool Gizmo_trackball::select_constraint(const Camera& cam, int px, int py)
{
    if(!_show){
        _selected_axis = NOAXIS;
        return false;
    }

    _old_frame = _frame;
    py = cam.height() - py; /////////// Invert because of opengl
    _clicked.x = px;
    _clicked.y = py;
    _selected_axis = BALL;
    return true;
}

// -----------------------------------------------------------------------------

TRS Gizmo_trackball::slide(const Camera& cam, int px, int py)
{
    py = cam.height() - py; /////////// Invert because of opengl
    if( _selected_axis == BALL) return trackball_rotation(cam, px, py);
    else                        return TRS();
}

// -----------------------------------------------------------------------------

TRS Gizmo_trackball::trackball_rotation(const Camera& cam, int px, int py)
{
    const Point_cu org    = Convs::to_point(_frame.get_translation());
    const Vec3_cu  picked = Vec3_cu((float)_clicked.x, (float)_clicked.y, 0.f);
    const Vec3_cu  curr   = Vec3_cu((float)px, (float)py, 0.f);
    const Vec3_cu  mouse_vec = curr - picked;

    if( mouse_vec.norm() < 0.001f ) return TRS();

#if 0
    // Pseudo trackBall
    Vec3_cu fx = cam.get_x();
    Vec3_cu fy = cam.get_y();

    Vec3_cu axis = fx * mouse_vec.y + fy * mouse_vec.x;
    axis.normalize();

    float norm  = mouse_vec.norm() / 60.f;
    float angle = fmodf( norm, 2.f* M_PI);

    // Rotation in local coordinates
    return TRS::rotation(_old_frame.fast_invert() * axis, angle);
#else

    Point_cu proj_org = cam.project(org);

    TrackBall ball(cam.width(), cam.height(), proj_org.x, proj_org.y, 0.5f);
    ball.set_picked_point(picked.x, picked.y);

    Vec3_cu eye_axis;
    float angle;
    ball.roll( (float*)&eye_axis, angle, curr.x, curr.y );
    Vec3_cu world_axis = cam.get_eye_transfo().fast_invert() * eye_axis;

    // Rotation in local coordinates
    return TRS::rotation(_old_frame.fast_invert() * world_axis, angle);
#endif

}

// -----------------------------------------------------------------------------

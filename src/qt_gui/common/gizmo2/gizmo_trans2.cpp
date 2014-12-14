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
#include "common/gizmo2/gizmo_trans2.hpp"

#include <limits>

#include "glsave.hpp"
#include "intersection.hpp"
#include "vec3_cu.hpp"
#include "vec2_cu.hpp"
#include "cuda_ctrl.hpp"
#include "conversions.hpp"

using namespace Cuda_ctrl;

// -----------------------------------------------------------------------------

static void draw_arrow(float radius, float length){
    GLUquadricObj* quad = gluNewQuadric();

    glLineWidth(1.f);
    //gluCylinder(quad, radius/3.f, radius/3.f, 0.8f * length, 10, 10);
    glBegin(GL_LINES);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 0.8f*length);
    glAssert( glEnd() );
    glTranslatef(0.f, 0.f, 0.8f * length);
    gluCylinder(quad, radius, 0.0f  , 0.2f * length, 10, 10);

    gluDeleteQuadric(quad);
}

void Gizmo_trans2::draw(const Camera& cam)
{
    if(!_show) return;

    glPushMatrix();
    GLEnabledSave save_light(GL_LIGHTING  , true, false);
    GLEnabledSave save_depth(GL_DEPTH_TEST, true, false);
    GLEnabledSave save_blend(GL_BLEND     , true, false);
    GLEnabledSave save_alpha(GL_ALPHA_TEST, true, false);
    GLEnabledSave save_textu(GL_TEXTURE_2D, true, false);

    const Vec3_cu org = _frame.get_translation();
    const float dist_cam =  !cam.is_ortho() ? (org-cam.get_pos()).norm() : _ortho_factor;

    glMultMatrixf(_frame.transpose().m);
    glScalef(dist_cam, dist_cam, dist_cam);

    /* Axis X */
    if(_constraint == AXIS && _selected_axis == X )
        glColor3f(1.f, 1.f, 0.f);
    else
        glColor3f(1.f, 0.f, 0.f);
    glPushMatrix();
    glRotatef(90.f, 0.f, 1.f, 0.f);
    draw_arrow(_radius, _length);
    glPopMatrix();

    /* Axis Y */
    if(_constraint == AXIS && _selected_axis == Y )
        glColor3f(1.f, 1.f, 0.f);
    else
        glColor3f(0.f, 1.f, 0.f);
    glPushMatrix();
    glRotatef(-90.f, 1.f, 0.f, 0.f);
    draw_arrow(_radius, _length);
    glPopMatrix();

    /* Axis Z */
    if(_constraint == AXIS && _selected_axis == Z )
        glColor3f(1.f, 1.f, 0.f);
    else
        glColor3f(0.f, 0.f, 1.f);
    glPushMatrix();
    draw_arrow(_radius, _length);
    glPopMatrix();

    glPopMatrix();
}

// -----------------------------------------------------------------------------

bool Gizmo_trans2::select_constraint(const Camera& cam, int px, int py)
{
    if(!_show){
        _selected_axis = NOAXIS;
        return false;
    }

    slide_from(_frame, Vec2i_cu(px, py) );
    // _old_frame = _frame;
    // _org.x = px;
    // _org.y = py;

    using namespace Inter;
    float t0, t1, t2;
    t0 = t1 = t2 = std::numeric_limits<float>::infinity();

    const Vec3_cu org = _frame.get_translation();
    const float dist_cam = !cam.is_ortho() ?
                           (org-cam.get_pos()).norm() : _ortho_factor;

    // TODO: use picking and not ray primitive intersection
    Ray_cu r = cam.cast_ray(px, py);

    const float s_radius = dist_cam * _radius;
    const float s_length = dist_cam * _length;
    Cylinder cylinder_x(s_radius, s_length, _frame.x(), org );
    Cylinder cylinder_y(s_radius, s_length, _frame.y(), org );
    Cylinder cylinder_z(s_radius, s_length, _frame.z(), org );
    Line cam_ray(r._dir, r._pos );
    Point nil;
    bool res = false;

//    if( cam->is_ortho() ){
//        Plane p(cam.get_dir(), cam.get_pos());
//        cylinder_x.project(p);
//        cylinder_y.project(p);
//        cylinder_z.project(p);
//    }

    res = res || cylinder_line( cylinder_x, cam_ray, nil, t0 );
    res = res || cylinder_line( cylinder_y, cam_ray, nil, t1 );
    res = res || cylinder_line( cylinder_z, cam_ray, nil, t2 );

    // TODO: use camera.project(org)
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    GLdouble cx, cy, cz;
    gluProject(org.x, org.y, org.z,
               modelview, projection, viewport,
               &cx, &cy, &cz);

    cy = cam.height() - cy;
    _pix_diff.x = (int)(cx-px);
    _pix_diff.y = (int)(cy-py);

    if( res )
    {
        _constraint = AXIS;
        if(t0 < t1 && t0 < t2){_axis = _frame.x(); _selected_axis = X; return true;}
        if(t1 < t0 && t1 < t2){_axis = _frame.y(); _selected_axis = Y; return true;}
        if(t2 < t1 && t2 < t0){_axis = _frame.z(); _selected_axis = Z; return true;}
    }

    return res;
}

// -----------------------------------------------------------------------------

TRS Gizmo_trans2::slide(const Camera& cam, int px, int py)
{
    Vec2i_cu pix(px, py);

    if(_constraint == AXIS && _selected_axis != NOAXIS)
    {
        if( (pix - _start_pix).norm() < 0.0001f )
            return TRS();
        else
            return slide_axis(_axis, cam, pix);
    }

    // TODO: why bother discriminating cases between X Y Z when we can use
    // the attribute _axis and _plane to. This gives rise to the use of useless
    // switchs ...
    // TODO: plane constraint :
    /*
        else
            switch(_selected_axis){
            case(XY): return slide_plane(_frame_z, cam, px, py); break;
            case(XZ): return slide_plane(_frame_y, cam, px, py); break;
            case(YZ): return slide_plane(_frame_x, cam, px, py); break;
            }
    */

    return TRS();
}

// -----------------------------------------------------------------------------

TRS Gizmo_trans2::slide_axis(Vec3_cu axis_dir, const Camera& cam, Vec2i_cu pix)
{
    pix += _pix_diff;

    // TODO:handle orthogonal projection
    Ray_cu  ray = cam.cast_ray(pix.x, pix.y);
    Vec3_cu up  = cam.get_y();
    Vec3_cu dir = cam.get_dir();

    // Find a plane passing through axis_dir and as parrallel as possible to
    // the camera image plane
    Vec3_cu ortho = dir.cross(axis_dir);
    Vec3_cu p_normal;
    if(ortho.norm() < 0.00001) p_normal = axis_dir.cross( up  );
    else                       p_normal = axis_dir.cross(ortho);


    // Intersection between that plane and the ray cast by the mouse
    const Vec3_cu org = _start_frame.get_org();
    Inter::Plane axis_plane(p_normal, org);
    Inter::Line l_ray(ray._dir, ray._pos);
    Vec3_cu slide_point;
    bool inter = Inter::plane_line(axis_plane, l_ray, slide_point);

    // Project on axis_dir the intersection point
    dir = axis_dir.normalized();
    slide_point = org + dir * dir.dot(slide_point-org);

    if(inter )
        return TRS::translation( slide_point-org );
    else
        return TRS();
}

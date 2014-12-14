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
#include "gl_skeleton.hpp"

#include "skeleton.hpp"
#include "std_utils.hpp"
#include "glsave.hpp"
#include "global_datas/globals.hpp"

// -----------------------------------------------------------------------------

GL_skeleton::GL_skeleton(const Skeleton* skel) :
    _skel(skel)
{  }

// -----------------------------------------------------------------------------

int GL_skeleton::nb_joints() const { return _skel->nb_joints(); }

// -----------------------------------------------------------------------------

static void custom_bone( float l  /*bone_length*/,
                         float js /*joint radius*/,
                         float ps /* parent joint radius*/)

{
    //l = 1.f;
    const float b0 = ps + (l / 10.f); // length of the base
    const float w0 = l / 15.f;        // width of the base
    // First pyramid
    glBegin( GL_TRIANGLE_FAN );
    {
        glVertex3f( ps, 0.f, 0.f );
        glVertex3f( b0, 0.f, -w0 );
        glVertex3f( b0, -w0, 0.f );
        glVertex3f( b0, 0.f,  w0 );
        glVertex3f( b0,  w0, 0.f );
        glVertex3f( b0, 0.f, -w0 );
    }
    glAssert( glEnd() );

    const float w1 = w0 / 3.f; // Width of the base at the opposite
    l = l-js;
    glBegin( GL_QUAD_STRIP );
    {
        glVertex3f( b0, 0.f, -w0 );// a
        glVertex3f(  l, 0.f, -w1 );// 0
        glVertex3f( b0, -w0, 0.f );// b
        glVertex3f(  l, -w1, 0.f );// 1
        glVertex3f( b0, 0.f,  w0 );// c
        glVertex3f(  l, 0.f,  w1 );// 2
        glVertex3f( b0,  w0, 0.f );// d
        glVertex3f(  l,  w1, 0.f );// 3
        glVertex3f( b0, 0.f, -w0 );// a
        glVertex3f(  l, 0.f, -w1 );// 0
    }
    glAssert( glEnd() );

    // The bone's cap is flat
    glBegin( GL_QUADS );
    {
        glVertex3f( l, 0.f, -w1 );
        glVertex3f( l, -w1, 0.f );
        glVertex3f( l, 0.f,  w1 );
        glVertex3f( l,  w1, 0.f );
    }
    glAssert( glEnd() );
}

// -----------------------------------------------------------------------------

static void draw_bone_body(const Transfo& b_frame,
                           const Point_cu& p0,
                           const Point_cu& p1,
                           float rad_joint,
                           float rad_pjoint)
{
    glPushMatrix();
#if 0
    Vec3_cu fx = (p1-p0).normalized(), fy, fz;
    fx.coordinate_system(fy, fz);
    Transfo tr( Mat3_cu(fx, fy, fz), p0.to_vector() );
    glMultMatrixf( tr.transpose().m );
#else
    Transfo tr = b_frame;
    glAssert( glMultMatrixf( tr.transpose().m ) );
#endif
    custom_bone( (p1-p0).norm(), rad_joint, rad_pjoint);
    glPopMatrix();
}

// -----------------------------------------------------------------------------

static void draw_frame(const Transfo& frame, float size_axis, bool color)
{
    GLLineWidthSave save_line_width( 2.0f );

    Transfo tr = frame.normalized();
    Point_cu pos = tr.get_translation().to_point();

    Point_cu dx = pos + tr.x() * size_axis;
    Point_cu dy = pos + tr.y() * size_axis;
    Point_cu dz = pos + tr.z() * size_axis;

    glBegin(GL_LINES);{
        // Local frame
        if(color) glColor4f(1.f, 0.f, 0.f, 1.f);
        glVertex3f(pos.x, pos.y, pos.z);
        glVertex3f(dx.x, dx.y, dx.z);
        if(color) glColor4f(0.f, 1.f, 0.f, 1.f);
        glVertex3f(pos.x, pos.y, pos.z);
        glVertex3f(dy.x, dy.y, dy.z);
        if(color) glColor4f(0.f, 0.f, 1.f, 1.f);
        glVertex3f(pos.x, pos.y, pos.z);
        glVertex3f(dz.x, dz.y, dz.z);
    }glAssert( glEnd() );

}

// -----------------------------------------------------------------------------

static void draw_joint( const Transfo& b_frame, float fc, bool use_circle)
{
    glAssert( glPushMatrix() );
    Transfo tr = b_frame;
    glAssert( glMultMatrixf( tr.transpose().m ) );
    if( !use_circle )
    {
        glScalef(fc, fc, fc);
        g_primitive_printer.draw( g_sphere_lr_vbo );
    }
    else
    {
        const float nfc = fc * 1.1f;
        glScalef(nfc, nfc, nfc);
        g_primitive_printer.draw( g_circle_lr_vbo );
        glRotatef(90.f, 1.f, 0.f, 0.f);
        g_primitive_printer.draw( g_circle_lr_vbo );
        glRotatef(90.f, 0.f, 1.f, 0.f);
        g_primitive_printer.draw( g_circle_lr_vbo );
    }
    glPopMatrix();
}

// -----------------------------------------------------------------------------

void GL_skeleton::draw_bone(int i, const Color& c, bool rest_pose, bool use_material, bool use_circle)
{
    const Bone* bone = _skel->get_bone(i);
    const float len  = bone->length();
    const float rad  = (len / 30.f);

    const Transfo b_frame = rest_pose ? _skel->bone_frame(i) : _skel->bone_anim_frame(i);
    const Point_cu org = bone->org();
    const Point_cu end = bone->end();

    glAssert( glMatrixMode(GL_MODELVIEW) );

    if(use_material) c.set_gl_state();
    draw_joint(b_frame, rad, use_circle );

    const float axis_size = 0.3f;

    // Draw bone body
    if( _skel->is_leaf(i) )
    {
        draw_frame(b_frame, axis_size, use_material);
        const int pt = _skel->parent( i );
        if(pt >= 0)
        {
            if(use_material) c.set_gl_state();
            draw_joint( b_frame, _skel->get_bone(pt)->length() / 50.f, use_circle);
        }
    }
    else
    {
        if(use_material){
            Color c = Color::pseudo_rand(i);
            glAssert( glColor4f(c.r, c.g, c.b, 1.f) );
        }
        draw_bone_body(b_frame, org, end, rad, rad);
    }
}

// -----------------------------------------------------------------------------

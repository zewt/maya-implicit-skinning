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
#include "camera.hpp"

#include "iostream"
#include "port_glew.h"
#include "quat_cu.hpp"

#include <conversions.hpp>
#include <cassert>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif


// -----------------------------------------------------------------------------

Camera::Camera() :
    _pos (0.f, 0.f, 0.f ),
    _dir (0.f, 0.f, 1.f),
    _x (1.f, 0.f, 0.f),
    _y (0.f, 1.f, 0.f),
    _fov(M_PI * 50.f / 180.f),
    _near(1.f),
    _far(300.f),
    _frustum_width(50.f),
    _ortho_proj(false),
    _width(100),
    _height(100),
    _offx(0),
    _offy(0)
{	}

// -----------------------------------------------------------------------------

void Camera::update_dir(float yaw, float pitch, float roll)
{
    Quat_cu rot = Quat_cu(_y, yaw) * Quat_cu(_x, pitch) * Quat_cu(_dir, roll);

    _dir = rot.rotate(_dir).normalized();
    _x   = rot.rotate(_x).normalized();
    _y   = rot.rotate(_y).normalized();

}

// -----------------------------------------------------------------------------

void Camera::set_fov_deg(float f)
{
    _fov = M_PI * f / 180.f;
}

// -----------------------------------------------------------------------------

void Camera::set_fov_rad(float f)
{
    _fov = f;
}

// -----------------------------------------------------------------------------

void Camera::set_dir(const Vec3_cu& dir)
{
    _dir = dir;
    _dir.normalize();
    Vec3_cu y(0.f,1.f,0.f);
    _x = y.cross( _dir );
    _x.normalize();
    _y = _dir.cross(_x);
}

// -----------------------------------------------------------------------------

/// update the position from a direction in local space
void Camera::update_pos(Vec3_cu mod_local_)
{
    _pos = _pos + (_x * mod_local_.x + _y * mod_local_.y + _dir * mod_local_.z);
}

// -----------------------------------------------------------------------------

void Camera::set_pos(float x, float y, float z)
{
    _pos = Vec3_cu(x,y,z);
}

// -----------------------------------------------------------------------------

/// dir and up must be orthogonal to each other
void Camera::set_dir_and_up(float dx, float dy, float dz,
                            float ux, float uy, float uz)
{
    _dir = Vec3_cu( dx, dy, dz);
    _y   = Vec3_cu( ux, uy, uz);
    _x   = _y.cross(_dir);
}

// -----------------------------------------------------------------------------

void Camera::lookat(float x, float y, float z)
{
    Vec3_cu dir = Vec3_cu(x, y, z) - _pos;
    set_dir( dir );
}

// -----------------------------------------------------------------------------

void Camera::lookat() const
{
    Vec3_cu o = _pos + _dir*2.f;
    gluLookAt(_pos.x, _pos.y, _pos.z,
                 o.x,    o.y,    o.z,
                _y.x,   _y.y,   _y.z);
}

// -----------------------------------------------------------------------------

void Camera::gl_mult_projection() const
{
    if(_ortho_proj)
    {
        float dx = _frustum_width * 0.5f;
        float dy = (float)_height * dx / (float)_width;
        glOrtho(-dx, dx, -dy, dy, _near, _far);
    }
    else
    {
        gluPerspective(_fov*180.0f / M_PI,
                       (float)_width/(float)_height,
                       _near,
                       _far);
    }
}

// -----------------------------------------------------------------------------

void Camera::roll(float theta)
{
    float ct = cosf(theta);
    float st = sinf(theta);
    Vec3_cu x = (_x * ct) + (_y * st);
    Vec3_cu y = (_y * ct) - (_x * st);
    _x = x;
    _y = y;
}

// -----------------------------------------------------------------------------

void Camera::local_strafe(float x, float y, float z)
{
    Vec3_cu v(x, y, z);
    update_pos(v);
}

// -----------------------------------------------------------------------------

Vec3_cu Camera::get_pos() const { return _pos; }

// -----------------------------------------------------------------------------

Vec3_cu Camera::get_dir() const { return _dir; }

// -----------------------------------------------------------------------------

Vec3_cu Camera::get_y() const{ return _y; }

// -----------------------------------------------------------------------------

Vec3_cu Camera::get_x() const { return _x; }

// -----------------------------------------------------------------------------

Transfo Camera::get_frame() const{
    return Transfo(Mat3_cu(_x, _y, _dir), _pos);
}

// -----------------------------------------------------------------------------

float Camera::get_frustum_width() const { return _frustum_width; }

// -----------------------------------------------------------------------------

void  Camera::set_frustum_width(float width)
{
    assert(width > 0.0001f);
    _frustum_width = width;
}

// -----------------------------------------------------------------------------

inline static float cot(float x){ return tan(M_PI_2 - x); }

Transfo Camera::get_proj_transfo() const
{
    if(!_ortho_proj)
    {
        // Compute projection matrix as describe in the doc of gluPerspective()
        const float f     = cot(_fov * 0.5f);
        const float ratio = (float)_width / (float)_height;
        const float diff  = _near - _far;

        return Transfo( f / ratio, 0.f,               0.f, 0.f,
                        0.f      , f  ,               0.f, 0.f,
                        0.f      , 0.f, (_far+_near)/diff, (2.f*_far*_near)/diff,
                        0.f      , 0.f,              -1.f, 0.f
                        );
    }
    else
    {
        const float dx = _frustum_width * 0.5f;
        const float dy = (float)_height * dx / (float)_width;
        // ------------
        // Compute projection matrix as describe in the doc of gluPerspective()
        const float l = -dx; // left
        const float r =  dx; // right
        const float t =  dy; // top
        const float b = -dy; // bottom

        Vec3_cu tr = Vec3_cu( -(r+l) / (r-l), -(t+b) / (t-b), - (_far+_near) / (_far-_near)  );

        return Transfo( 2.f / (r-l), 0.f          ,  0.f               , tr.x,
                        0.f        , 2.f / (t-b)  ,  0.f               , tr.y,
                        0.f        , 0.f          , -2.f / (_far-_near), tr.z,
                        0.f        , 0.f          ,  0.f               , 1.f
                        );
    }
}

// -----------------------------------------------------------------------------

Transfo Camera::get_eye_transfo() const
{
    /*
    We use the glulookat implementation :

    Let :
                    centerX - eyeX
                F = centerY - eyeY
                    centerZ - eyeZ

               Let UP be the vector (upX, upY, upZ).

               Then normalize as follows: f = F/ || F ||

               UP' = UP/|| UP ||

               Finally, let s = f X UP', and u = s X f.

               M is then constructed as follows:

                     s[0]    s[1]    s[2]    0
                     u[0]    u[1]    u[2]    0
                M = -f[0]   -f[1]   -f[2]    0
                      0       0       0      1

               and gluLookAt is equivalent to

               glMultMatrixf(M);
               glTranslated (-eyex, -eyey, -eyez);
    */

    // Implementation :
    const Vec3_cu eye = get_pos();
    const Vec3_cu f   = get_dir().normalized();
    const Vec3_cu up  = get_y().normalized();

    const Vec3_cu s = f.cross( up );
    const Vec3_cu u = s.cross( f  );

    const Transfo trans = Transfo::translate( -eye );

    return  Transfo(  s.x,  s.y,  s.z, 0.f,
                      u.x,  u.y,  u.z, 0.f,
                     -f.x, -f.y, -f.z, 0.f,
                      0.f,  0.f,  0.f, 1.f) * trans;
}

// -----------------------------------------------------------------------------

Transfo Camera::get_viewport_transfo() const
{
    Transfo tr  = Transfo::translate( 1.f, 1.f, 1.f );
    Transfo sc  = Transfo::scale((float)_width * 0.5f, (float)_height * 0.5f, 0.5f);
    Transfo off = Transfo::translate((float)_offx, (float)_offy, 0.f );

    return off * sc * tr;
}

// -----------------------------------------------------------------------------

Point_cu Camera::project(const Point_cu& p) const
{
    return get_viewport_transfo() * get_proj_transfo().project( get_eye_transfo() * p);
}

// -----------------------------------------------------------------------------

Point_cu Camera::un_project(const Point_cu& p) const
{
    Transfo t = (get_viewport_transfo() * get_proj_transfo() * get_eye_transfo()).full_invert();
    return  t.project( p ); // Multiply and do perspective division
}

// -----------------------------------------------------------------------------

Ray_cu Camera::cast_ray(int px, int py) const
{
    Ray_cu ray;
    Vec3_cu cam_dir = _dir;
    Vec3_cu cam_pos = _pos;
    Vec3_cu cam_hor = -_x;
    Vec3_cu cam_ver = -_y;

    if(_ortho_proj)
    {
        ray.set_dir(cam_dir);
        float dx = (px * 1.f/_width - 0.5f) * _fov;
        float dy = (py * 1.f/_height - 0.5f) * _fov * (_height *  1.f/_width);
        ray.set_pos( Convs::to_point(cam_pos + cam_hor * dx + cam_ver * dy) );
    }
    else
    {
        Vec3_cu dep =  cam_dir * (0.5f/tanf(0.5f*_fov));
        ray.set_pos( Convs::to_point(cam_pos) );
        Vec3_cu ver = cam_ver * (1.f / _height);
        Vec3_cu hor = cam_hor * (1.f / _height);
        Vec3_cu dir = (dep + hor * (px - _width/2.f)	+ ver * (py - _height/2.f));
        ray.set_dir(dir.normalized());
    }
    return ray;
}

// -----------------------------------------------------------------------------

void Camera::print() const
{
    std::cout << "position"  << _pos.x << "," << _pos.y << "," << _pos.z << "\n";
    std::cout << "direction" << _dir.x << "," << _dir.y << "," << _dir.z << "\n";
    std::cout << "up vector" << _y.x   << "," << _y.y   << "," << _y.z   << "\n";
    std::cout << "x axis"    << _x.x   << "," << _x.y   << "," << _x.z   << std::endl;
}

// -----------------------------------------------------------------------------

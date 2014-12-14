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
#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "ray_cu.hpp"
#include "vec3_cu.hpp"
#include "transfo.hpp"

/** @class Camera
  @brief Handles camera movements, and various related parameters
*/

class Camera {
public:

    Camera();

    /// update the direction from a rotation
    /// @param yaw_    rotation around y
    /// @param pitch_  rotation around x
    void update_dir(float yaw, float pitch, float roll);

    /// Set the camera aperture in degrees
    void set_fov_deg(float f);

    /// Set the camera aperture in radians
    void set_fov_rad(float f);

    void set_dir(const Vec3_cu& dir);

    /// update the position from a direction in local space
    void update_pos(Vec3_cu mod_local_);

    void set_pos(float x, float y, float z);

    /// dir and up must be orthogonal to each other
    void set_dir_and_up (float dx, float dy, float dz,
                         float ux, float uy, float uz);

    void lookat(float x, float y, float z);

    /// This call gluLookAt
    void lookat() const;

    /// Multiply the current OpenGL matrix with the projection matrix of the
    /// camera
    void gl_mult_projection() const;

    void roll(float theta);

    void local_strafe(float x, float y, float z);

    Vec3_cu get_pos() const;

    Vec3_cu get_dir() const;

    Vec3_cu get_y() const;

    Vec3_cu get_x() const;

    Transfo get_frame() const;

    float get_frustum_width() const;
    void  set_frustum_width(float width);

    /// Compute the projection matrix. (as in OpenGL from eye coordinates
    /// to normalized device coordinates)
    Transfo get_proj_transfo() const;

    /// Get the view matrix (As computed with gluLookAt from model coordinates
    /// to eye coordinates
    Transfo get_eye_transfo() const;

    /// Get the viewport matrix (as in OpenGL from normalized device coordinates
    /// to window coordinates)
    /// @note z is mapped between [0 1] as in the default value of openGL
    /// for glDepthRange()
    Transfo get_viewport_transfo() const;

    /// Project 'p' in world coordinates to the camera image plane
    /// (in window coordinates)
    /// @return p' = viewport_transfo * proj_transfo * eye_transfo * p
    Point_cu project(const Point_cu& p) const;

    /// unProject 'p' in window coordinates to world coordinates
    /// @return p' = (viewport_transfo * proj_transfo * eye_transfo)^-1 * p
    Point_cu un_project(const Point_cu& p) const;

    /// Cast a ray from camera : px, py are the pixel position you want
    /// to cast a ray from
    Ray_cu cast_ray(int px, int py) const;

    int width()  const { return _width;  }
    int height() const { return _height; }

    float get_near() const { return _near;  }
    float get_far()  const { return _far;   }

    float fovy() const { return _fov; }

    bool is_ortho() const { return _ortho_proj; }

    void set_ortho(bool s) { _ortho_proj = s; }

    void set_near(float n){ _near = n; }

    void set_far(float f){ _far = f; }

    void set_viewport(int x, int y, int w, int h){
        _width  = w;
        _height = h;
        _offy   = y;
        _offx   = x;
    }

    void print() const;

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
private:
    Vec3_cu _pos;    ///< position
    Vec3_cu _dir;    ///< sight direction (z axis)
    Vec3_cu _x, _y;  ///< other vectors to get the frame

    float _fov;           ///< openGl opening angle along Y axis (in radian)
    float _near;          ///< near plane distance
    float _far;           ///< far plane distance
    float _frustum_width; ///< frustum width in orthogonal projection

    bool _ortho_proj;     ///< does orthogonal projection enabled ?

    int _width;  ///< width of the camera viewport
    int _height; ///< height of the camera viewport

    int _offx;   ///< x offset of the camera viewport
    int _offy;   ///< y offset of the camera viewport
};

#endif

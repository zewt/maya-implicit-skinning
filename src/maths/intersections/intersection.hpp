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
#ifndef INTERSECTION_H_
#define INTERSECTION_H_

/**
  @namespace Inter
  @brief various intersection calculus.

*/

#include "vec3_cu.hpp"
#include "point_cu.hpp"

// =============================================================================
namespace Inter {
// =============================================================================

typedef Vec3_cu Vec;
typedef Vec3_cu Point;

// -----------------------------------------------------------------------------

struct Line {

    IF_CUDA_DEVICE_HOST
    Line(Vec dir, const Point& org) {
        this->dir = dir.normalized();
        this->org = org;
    }

    Vec   dir;  ///< Line direction
    Point org;  ///< Line origine
};

// -----------------------------------------------------------------------------

struct Plane {

    IF_CUDA_DEVICE_HOST
    Plane(Vec normal, const Point& org){
        this->normal = normal.normalized();
        this->org    = org;
    }

    Vec   normal;  ///< Normale to plane
    Point org;     ///< Plane origine
};

// -----------------------------------------------------------------------------

struct Sphere {

    IF_CUDA_DEVICE_HOST
    Sphere(float radius, const Point& org){
        this->radius = radius;
        this->org    = org;
    }

    float radius; ///< sphere radius
    Point org;    ///< sphere origine
};

// -----------------------------------------------------------------------------

/// A cylinder start from org and stop at org+dir*length
struct Cylinder {

    IF_CUDA_DEVICE_HOST
    Cylinder(float radius, float length, const Vec& dir, const Point& org){
        this->radius = radius;
        this->org    = org;
        this->dir    = dir.normalized();
        this->length = length;
    }

    float length; ///< cylinder length
    float radius; ///< cylinder radius
    Point org;    ///< cylinder origine
    Vec   dir;    ///< cylinder direction
};

// -----------------------------------------------------------------------------

struct Triangle {

    IF_CUDA_DEVICE_HOST
    Triangle(const Point& p0, const Point& p1, const Point& p2){
        p[0] = p0; p[1] = p1; p[2] = p2;
    }

    Point p[3];
};

// -----------------------------------------------------------------------------

/// @param res  intersection location
/// @return true if intersect false otherwise
IF_CUDA_DEVICE_HOST static inline
bool plane_line(const Plane& p, const Line& l, Point& res);

/// @param res  nearest intersection location
IF_CUDA_DEVICE_HOST static inline
bool sphere_line(const Sphere& s, const Line& l, Point& res, float& t);

/// @param res  nearest intersection location
IF_CUDA_DEVICE_HOST static inline
bool cylinder_line(const Cylinder& c, const Line& l, Point& res, float& t);

/// self intersection of a line with a line
/// @param res    the intersection position if there is any
/// @return true  if intersect
IF_CUDA_DEVICE_HOST static inline
bool line_line( const Line& line1, const Line& line2, Point& res);

/// self intersection of a triangle with a triangle
/// @param res1, res2 : the intersection segment if there is any
/// @param coplanar : if triangles are coplanar
/// @param eps : the chosen epsilon threshold to detect intersection.
/// set to zero to disable this test (but less robust).
/// @return true if intersects
/// @note If you wish to find intersection between two mesh or self-intersection
/// you might want to avoid degenerate cases. To do so add some random noise
/// to your vertex positions. The noise norm must be greater than 'eps' and it
/// will statistically guarantee to find a closed curve of intersection (if the
/// mesh are closed and manifold). Otherwise you'll have to do more testing,
/// knowing that it can take a while to produce a robust algorithm ...
IF_CUDA_DEVICE_HOST static inline
bool tri_tri(const Triangle& tri1, const Triangle& tri2,
             Point& res1, Point& res2,
             bool &coplanar,
             float eps = 0.000001f);

}// Inter ======================================================================

#include "intersection.inl"

#endif // INTERSECTION_H_




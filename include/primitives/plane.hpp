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
#ifndef PLANE_HPP__
#define PLANE_HPP__

#include "point_cu.hpp"
#include "vec3_cu.hpp"
//#include "implicit_types.hpp"
#include "distance_field.hpp"

/// @brief implicit plane
struct Plane{
    __device__ __host__
    inline Plane() {};

    __device__ __host__
    inline Plane(const Vec3_cu& normal, const Point_cu& b, float r) :
        n(normal.normalized()), radius(r / Field::field_to_distance(0.5f,1.f)),
        offset(pt_vec_dot(b,n) - Field::field_to_distance(0.5f,r / Field::field_to_distance(0.5f,1.f))) {};

    __device__ __host__
    float	f(const Point_cu& p) const;

    __device__ __host__
    inline Vec3_cu	gf(const Point_cu& p) const;

    __device__ __host__
    inline float fngf(Vec3_cu& gf, const Point_cu& p) const;

    __device__ __host__
    static inline float pt_vec_dot(const Point_cu& p, const Vec3_cu& v);

private:
    Vec3_cu n;
    float radius;
    float offset;
};


/// @brief implicit plane using flatten & symetric funtion ( distance )
struct S_Plane{
    __device__ __host__
    inline S_Plane() {};

    __device__ __host__
    inline S_Plane(const Vec3_cu& normal, const Point_cu& b, float r) :
        n(normal.normalized()), radius(r), p(b),
        recal(pt_vec_dot(p,n) - r),
        offset(radius) {};

    __device__ __host__
    float	f(const Point_cu& p) const;

    __device__ __host__
    inline Vec3_cu	gf(const Point_cu& p) const;

    __device__ __host__
    inline float fngf(Vec3_cu& gf, const Point_cu& p) const;

    __device__ __host__
    static inline float pt_vec_dot(const Point_cu& p, const Vec3_cu& v);


    void set_radius(float r) {
        radius = r;
        if (offset > radius)
            offset = radius; ///< potential offset around surface
        recal = pt_vec_dot(p,n) - r;
    }

    float get_radius() {
        return radius;
    }

    Vec3_cu n;
    float radius;
    Point_cu p;
    float recal;  ///< offset for plane recalage
    float offset; ///< potential offset around surface
};

#include "plane.inl"

#endif // PLANE_HPP__

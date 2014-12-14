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
#ifndef CYLINDER_HPP__
#define CYLINDER_HPP__

#include "point_cu.hpp"
#include "vec3_cu.hpp"
#include "distance_field.hpp"

/**
 * @brief implicit cylinder
    @warning don't add more attributes without changing skeleton_tex.hpp
    (this implicit primitives is stored into texture)
*/
struct Cylinder{
    __device__ __host__
    inline Cylinder() : o(Point_cu()), radius(1.f), v(Vec3_cu::unit_x()), length(v.norm()) {}

    __device__ __host__
    inline Cylinder(const Point_cu& p1, const Point_cu& p2, float r):
        o(p1),
        radius(r/Field::field_to_distance(POTENTIAL_ISO,1.f)),
        v(p2-p1),
        length((p2-p1).norm())
    { }

    __device__ __host__
    inline Cylinder(const Cylinder& c):
        o(c.o),
        radius(c.radius),
        v(c.v),
        length(c.length)
    { }

    __device__ __host__
    float	f(const Point_cu& p) const;

    __device__ __host__
    inline Vec3_cu	gf(const Point_cu& p) const;

    __device__ __host__
    inline float fngf(Vec3_cu& gf, const Point_cu& p) const;

    inline __host__ float get_radius(){
        return (radius*Field::field_to_distance(POTENTIAL_ISO,1.f));
    }

    inline __host__ float set_radius(float rad){
        return radius = rad / Field::field_to_distance(POTENTIAL_ISO,1.f);
    }

    inline __host__ float get_potential_radius(){
        return radius;
    }

    inline __host__ float set_potential_radius(float rad){
        return radius = rad;
    }

    inline __host__ Point_cu get_origin(){
        return o;
    }

    inline __host__ void set_origin(const Point_cu &p){
        o = p;
    }

    inline __host__ Vec3_cu get_dir(){
        return v;
    }

    inline __host__ void set_dir(const Vec3_cu &d){
        v = d;
        length = v.norm();
    }

private:
    Point_cu o;   ///< cylinder's origin
    float radius; ///< cylinder's radius
    Vec3_cu v;  ///< cylinder's direction (not normalized)
    float length; ///< norm of vector v
};

struct S_Cylinder {
    __device__ __host__
    inline S_Cylinder() :
        o(Point_cu()),
        radius(1.f),
        offset(radius),
        v(Vec3_cu::unit_x()),
        length(v.norm())
    { }

    __device__ __host__
    inline S_Cylinder(const Point_cu& p1, const Point_cu& p2, float r):
        o(p1),
        radius(r),
        offset(radius),
        v(p2-p1),
        length((p2-p1).norm())
    { }

    __device__ __host__
    inline S_Cylinder(const S_Cylinder& c):
        o(c.o),
        radius(c.radius),
        offset(c.offset),
        v(c.v),
        length(c.length)
    { }

    __device__ __host__
    float f(const Point_cu& p) const;

    __device__ __host__
    inline Vec3_cu gf(const Point_cu& p) const;

    __device__ __host__
    inline float fngf(Vec3_cu& gf, const Point_cu& p) const;

    inline __host__ float get_radius(){
        return radius;
    }

    __host__
    inline void set_radius(float rad){
        radius = rad;
        if (offset > radius)
            offset = radius;
    }

    __host__
    inline __host__ float get_potential_radius(){
        return radius + offset;
    }

    __host__
    inline __host__ float get_potential_offset(){ return offset; }

    __host__
    inline float set_potential_offset(float o){ return offset = o; }

    __host__
    inline Point_cu get_origin(){ return o; }

    inline void set_origin(const Point_cu &p){ o = p; }

    __host__
    inline Vec3_cu get_dir(){ return v; }

    __host__
    inline void set_length(float l){ v = v.normalized()*l; length = l; }

private:
    Point_cu o;   ///< cylinder's origin
    float radius; ///< cylinder's surface radius
    float offset; ///< potentiel offset around segment
    Vec3_cu v;  ///< cylinder's direction (not normalized)
    float length; ///< norm of vector v
};

#include "cylinder.inl"

#endif // CYLINDER_HPP__

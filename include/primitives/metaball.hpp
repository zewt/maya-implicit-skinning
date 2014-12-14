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
#ifndef METABALL_HPP__
#define METABALL_HPP__

#include "vec3_cu.hpp"
#include "point_cu.hpp"
#include "distance_field.hpp"

struct Metaball{
    __device__ __host__
    inline Metaball() {};

    __device__ __host__
    inline Metaball(const Point_cu& p, float r): pos(p), radius(r/Field::field_to_distance(0.5f, 1.f)) {};

    __device__ __host__
    float	f(const Point_cu& p) const;

    __device__ __host__
    Vec3_cu gf(const Point_cu& p) const;

    __device__ __host__
    float	fngf(Vec3_cu& gf, const Point_cu& p) const;

    __host__
    void rotate(const Point_cu& p, const Vec3_cu& axis_, float angle);

    __host__
    void translate(const Vec3_cu& v);

    __host__
    void set_pos(const Point_cu& p);

    __host__
    Point_cu get_pos() const;

    template <class OtherPrim>
    __host__
    bool has_merged(const OtherPrim& p, float scale) const;

private:
    Point_cu pos;
    float radius;
};

#include "metaball.inl"

#endif // METABALL_HPP__

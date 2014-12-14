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
#include "distance_field.hpp"

__device__ __host__
inline float Metaball::f(const Point_cu& p) const {
    float d2 = pos.distance_squared(p);
    return Field::distance_squared_to_field(d2, radius);
}

__device__ __host__
inline Vec3_cu Metaball::gf(const Point_cu& p) const {
    float d2 = pos.distance_squared(p);
    float d = sqrtf(d2);
    float df = Field::field_derivative_from_distance(d, radius);
    Vec3_cu grad_d = (p - pos) * (1.f / d);
    return grad_d * df;
}

__device__ __host__
inline float Metaball::fngf(Vec3_cu& gf, const Point_cu& p) const {
    float d2 = pos.distance_squared(p);
    float d = sqrtf(d2);
    float2 fndf = Field::distance_to_field_and_derivative(d, radius);
    Vec3_cu grad_d = (p - pos) * (1.f / d);
    gf = grad_d * fndf.y;
    return fndf.x;
}

// -----------------------------------------------------------------------------

__host__
void Metaball::rotate(const Point_cu& p, const Vec3_cu& axis_, float angle){
    Vec3_cu axis = axis_.normalized();
    Mat3_cu rot = Mat3_cu :: rotate(axis, angle);
    Vec3_cu d = pos - p;
    pos = p + rot * d;
}

__host__
void Metaball::translate(const Vec3_cu& v){
    pos = pos + v;
}

__host__
void Metaball::set_pos(const Point_cu& p){
    pos = p;
}

__host__
Point_cu Metaball::get_pos() const {
    return pos;
}

template <class OtherPrim>
__host__
bool Metaball::has_merged(const OtherPrim& p, float scale) const {
    Point_cu ppos = p.get_pos();
    float dist = ppos.distance_squared(pos) * scale * scale;
    if(dist < radius * radius * 0.25f)
        return true;
    return false;
}

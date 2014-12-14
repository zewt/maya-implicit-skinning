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
//==============================================================================
// Class Cylinder
//==============================================================================

__device__ __host__
inline float	Cylinder::f(const Point_cu& p) const {
    //compute projection on cylinder axis
    Vec3_cu po = o - p;
    float x = - po.dot(v)/(length*length);
    x = fminf(1.f, fmaxf(0.f, x));
    Point_cu proj = o + v * x;
    //compute potential based on distance to projection
    float d2 = proj.distance_squared(p);
    return Field::distance_squared_to_field(d2, radius);
}

//------------------------------------------------------------------------------

__device__ __host__
inline Vec3_cu	Cylinder::gf(const Point_cu& p) const {
    //compute projection on cylinder axis
    Vec3_cu po = o - p;
    float x = - po.dot(v)/(length*length);
    x = fminf(1.f, fmaxf(0.f, x));
    Point_cu proj = o + v * x;
    //compute gradient of the potential
    float d2 = proj.distance_squared(p);
    float d = sqrtf(d2);
    float df = Field::field_derivative_from_distance(d, radius);
    Vec3_cu grad_d = (p - proj) * (1.f / d);
    return grad_d * df;
}

//------------------------------------------------------------------------------

__device__ __host__
inline float Cylinder::fngf(Vec3_cu& gf, const Point_cu& p) const {
    //compute projection on cylinder axis
    Vec3_cu po = o - p;
    float x = - po.dot(v)/(length*length);
    x = fminf(1.f, fmaxf(0.f, x));
    Point_cu proj = o + v * x;
    //compute potential based on distance to projection
    float d = sqrtf(proj.distance_squared(p));
    float2 fndf = Field::distance_to_field_and_derivative(d, radius);
    Vec3_cu grad_d = (p - proj) * (1.f / d);
    gf =  grad_d * fndf.y;
    return fndf.x;
}


//==============================================================================
// Class S_Cylinder
//==============================================================================

__device__ __host__
inline float S_Cylinder::f(const Point_cu& p) const {
    //compute projection on cylinder axis
    Vec3_cu po = o - p;
    float x = - po.dot(v)/(length*length);
    x = fminf(1.f, fmaxf(0.f, x));
    Point_cu proj = o + v * x;
    //compute potential based on distance to projection
    float d = (proj-p).norm();
    return Field::distance_to_field_flatten(d, radius, offset);
}

//------------------------------------------------------------------------------

__device__ __host__
inline Vec3_cu S_Cylinder::gf(const Point_cu& p) const {
    //compute projection on cylinder axis
    Vec3_cu po = o - p;
    float x = - po.dot(v)/(length*length);
    x = fminf(1.f, fmaxf(0.f, x));
    Point_cu proj = o + v * x;
    //compute gradient of the potential
    float d = (proj-p).norm();
    float df = Field::field_derivative_from_distance_flatten(d, radius, offset);
    Vec3_cu grad_d = (p - proj) * (1.f / d);
    return grad_d * df;
}

//------------------------------------------------------------------------------

__device__ __host__
inline float S_Cylinder::fngf(Vec3_cu& gf, const Point_cu& p) const {
    //compute projection on cylinder axis
    Vec3_cu po = o - p;
    float x = - po.dot(v)/(length*length);
    x = fminf(1.f, fmaxf(0.f, x));
    Point_cu proj = o + v * x;
    //compute potential based on distance to projection
    float d = (proj-p).norm();
    float2 fndf = Field::distance_to_field_and_derivative_flatten(d, radius, offset);
    Vec3_cu grad_d = (p - proj) * (1.f / d);
    gf =  grad_d * fndf.y;
    return fndf.x;
}

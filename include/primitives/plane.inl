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
__device__ __host__
inline float	Plane::f(const Point_cu& p) const {
	float d = fmaxf(0.f, pt_vec_dot(p, n) - offset);
	return Field::distance_to_field(d, radius);
}

__device__ __host__
inline Vec3_cu	Plane::gf(const Point_cu& p) const {
	float d = fmaxf(0.f,pt_vec_dot(p, n) - offset);
	float df = Field::field_derivative_from_distance(d, radius);
	return n * df;
}

__device__ __host__
inline float Plane::fngf(Vec3_cu& gf, const Point_cu& p) const {
	float d = fmaxf(0.f,pt_vec_dot(p, n) - offset);
	float2 fndf = Field::distance_to_field_and_derivative(d, radius);
	gf = n * fndf.y;
	return fndf.x;
}


__device__ __host__
inline float Plane::pt_vec_dot(const Point_cu& p, const Vec3_cu& v){
	return p.x * v.x + p.y * v.y + p.z * v.z;
}


// =============================================================================
// Struct S_Plane
// =============================================================================

__device__ __host__
inline float S_Plane::f(const Point_cu& p) const {
        float d = fmaxf(0.f, pt_vec_dot(p, n) - recal);
        return Field::distance_to_field_flatten(d, radius, radius);
}

__device__ __host__
inline Vec3_cu S_Plane::gf(const Point_cu& p) const {
        float d = fmaxf(0.f,pt_vec_dot(p, n) - recal);
        float df = Field::field_derivative_from_distance_flatten(d, radius, radius);
        return n * df;
}

__device__ __host__
inline float S_Plane::fngf(Vec3_cu& gf, const Point_cu& p) const {
        float d = fmaxf(0.f,pt_vec_dot(p, n) - recal);
        float2 fndf = Field::distance_to_field_and_derivative_flatten(d, radius, radius);
        gf = n * fndf.y;
        return fndf.x;
}


__device__ __host__
inline float S_Plane::pt_vec_dot(const Point_cu& p, const Vec3_cu& v){
        return p.x * v.x + p.y * v.y + p.z * v.z;
}

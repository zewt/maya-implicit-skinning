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
inline float Cube::f(const Point_cu& p) const {
	bool inside;
	Point_cu proj = project_on_cube(p, inside);
	float d = sqrtf(proj.distance_squared(p));
	d = inside ? -d : d;
	float d_offset = Field::field_to_distance(0.5f, 1.f);
	d += d_offset;
	return Field::distance_to_field(d, 1.f);
}


__device__ __host__
inline Vec3_cu Cube::gf(const Point_cu& p) const {
	bool inside;
	Point_cu proj = project_on_cube(p, inside);
	Vec3_cu gd = proj - p;
	float nm = gd.norm();
	nm = inside ? -nm : nm;
	float d = nm;
	float d_offset = Field::field_to_distance(0.5f, 1.f);
	d += d_offset;
	float df = Field::field_derivative_from_distance(d, 1.f);
	return gd * (df / nm);
	//return Vec3_cu(1.f, 0.f, 0.f) * df;
}

__device__ __host__
inline float Cube::fngf(Vec3_cu& gf, const Point_cu& p) const {
	bool inside;
	Point_cu proj = project_on_cube(p, inside);
	Vec3_cu gd = proj - p;
	float nm = gd.norm();
	nm = inside ? -nm : nm;
	float d = nm;
	float d_offset = Field::field_to_distance(0.5f, 1.f);
	d += d_offset;
	float2 fndf = Field::distance_to_field_and_derivative(d, 1.f);
	gf = gd * (fndf.y / nm);
	return fndf.x;
}


__device__ __host__
inline Point_cu Cube::project_on_cube(const Point_cu& p, bool& inside) const {
	Point_cu res;
	res.x = fmaxf(pos.x - size, fminf(pos.x + size, p.x));
	res.y = fmaxf(pos.y - size, fminf(pos.y + size, p.y));
	res.z = fmaxf(pos.z - size, fminf(pos.z + size, p.z));
	inside = (res.x == p.x & res.y == p.y & res.z == p.z);
	if(inside){
		Vec3_cu df = p - pos;
		float mx = fmaxf(fabsf(df.x), fmaxf(fabsf(df.y), fabsf(df.z)));
		df.x = (fabsf(df.x) == mx)?df.x * (size-mx) / mx:0.f;
		df.y = (fabsf(df.y) == mx)?df.y * (size-mx) / mx:0.f;
		df.z = (fabsf(df.z) == mx)?df.z * (size-mx) / mx:0.f;
		res = p + df;
	}
	return res;
}

__device__ __host__
inline Point_cu Cube::center_project_on_cube(const Point_cu& p, bool& inside) const {
	Vec3_cu op = p - pos;
	float rd = fmaxf(fabsf(op.x), fmaxf(fabsf(op.y), fabsf(op.z)));
	float ds = size / rd;
	inside = ds > 1.f;
	return pos + op * ds;
}
/*
__device__ __host__
Point_cu center_project_on_cube_and_normal(const Point_cu& p, bool& inside, Vec3_cu& nm) const {
	Vec3_cu op = p - pos;
	float rd = fmaxf(fabsf(op.x), fabsf(op.y), fabsf(op.z));
	nm.x = (fabsf(op.x) == rd)?op.x:0.f;
	nm.y = (fabsf(op.y) == rd)?op.y:0.f;
	nm.z = (fabsf(op.z) == rd)?op.z:0.f;
	float ds = size / rd;
	inside = ds > 1.f;
	return pos + op * ds;
}
*/

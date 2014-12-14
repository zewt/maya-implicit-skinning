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
#ifndef CUBE_HPP__
#define CUBE_HPP__

#include "vec3_cu.hpp"
#include "point_cu.hpp"
#include "implicit_types.hpp"

/// @brief implicit cube
struct Cube{
    __device__ __host__
    inline Cube() {};

    __device__ __host__
    inline Cube(const Point_cu& p, float s): pos(p), size(s) {};

    __device__ __host__
    float	f(const Point_cu& p) const;

    __device__ __host__
    Vec3_cu gf(const Point_cu& p) const;

    __device__ __host__
    float	fngf(Vec3_cu& gf, const Point_cu& p) const;

    __device__ __host__
    Point_cu project_on_cube(const Point_cu& p, bool& inside) const;

    __device__ __host__
    Point_cu center_project_on_cube(const Point_cu& p, bool& inside) const;

private:
    Point_cu pos;
    float size;
};


#include "cube.inl"

#endif // CUBE_HPP__

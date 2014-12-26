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
#ifndef BLENDING_FUNCTIONS_HPP__
#define BLENDING_FUNCTIONS_HPP__

#include "uclean.hpp"
#include "ucommon.hpp"
#include "ultimate.hpp"
#include "dyn_operators.hpp"


/** @brief Blending operators for implicit surfaces

  operators must follow this implementation :

  @code

struct Binnary_operator{
    __device__ __host__
    static float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ __host__
    static Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ __host__
    static float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);
};

   @endcode


*/

#endif // BLENDING_FUNCTIONS_HPP__

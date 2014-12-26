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
#ifndef MATH_CU_HPP__
#define MATH_CU_HPP__

/** @file math_cu.hpp

  This header allows to use maths functions in both and nvcc and gcc compiler

*/


#ifdef __CUDACC__

#include <cuda_runtime.h>

#else

#include <cmath>
inline float fmaxf(float a, float b){ return a > b ? a : b; }

inline float fminf(float a, float b){ return a < b ? a : b; }

inline int max(int a, int b){ return a > b ? a : b; }

inline int min(int a, int b){ return a < b ? a : b; }
#endif


#endif // MATH_CU_HPP__

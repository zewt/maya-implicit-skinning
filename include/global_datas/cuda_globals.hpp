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
#ifndef CUDA_GLOBALS_HPP__
#define CUDA_GLOBALS_HPP__

#include "vec3_cu.hpp"
#include "point_cu.hpp"

struct Graph;
struct Skeleton;
struct Animesh;

/** Various global variables used in cuda source files
 */

/// The graph of the current skeleton
extern Graph* g_graph;

/// The current skeleton
extern Skeleton* g_skel;

/// The current animated mesh
extern Animesh* g_animesh;

#endif // CUDA_GLOBALS_HPP__

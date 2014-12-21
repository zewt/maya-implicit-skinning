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
#ifndef GLOBAL_HPP__
#define GLOBAL_HPP__

#include <string>
#include "port_glew.h"
#include "macros.hpp"
#include "mesh.hpp"
#include "gl_mesh.hpp"

#include "blending_env_type.hpp"

/// Textures used to display controller and operator frames
extern GLuint g_op_frame_tex;
extern GLuint g_op_tex;

/// Path to store various caches (mostly blending operators)
extern std::string g_cache_dir;

/// The current mesh
extern Mesh* g_mesh;

#endif // GLOBAL_HPP__

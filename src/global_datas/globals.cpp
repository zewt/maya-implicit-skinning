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
 #include "globals.hpp"

Blending_env::Op_id CUSTOM_TEST = -1;

// -----------------------------------------------------------------------------

Point_cache_file* g_anim_cache = 0;

// -----------------------------------------------------------------------------

VBO_primitives g_primitive_printer;
Prim_id g_sphere_lr_vbo;
Prim_id g_sphere_vbo;
Prim_id g_circle_vbo;
Prim_id g_arc_circle_vbo;
Prim_id g_circle_lr_vbo;
Prim_id g_arc_circle_lr_vbo;
Prim_id g_grid_vbo;
Prim_id g_cylinder_vbo;
Prim_id g_cylinder_cage_vbo;
Prim_id g_cube_vbo;

GLuint g_gl_quad;

// -----------------------------------------------------------------------------

GLuint g_gl_Tex[4];
GLuint g_ctrl_frame_tex;
GLuint g_op_frame_tex;
GLuint g_op_tex;

// -----------------------------------------------------------------------------

Shader_prog* g_points_shader = 0;

namespace Tex_units{
    const int KD   = 3;
    const int KS   = 4;
    const int BUMP = 5;
}

// -----------------------------------------------------------------------------

//std::string write_dir = "/export/home/magritte/vaillant/ppm_img";
std::string g_cache_dir  = "./resource/app_cache";
std::string g_config_dir = "./resource/app_config";
std::string g_icons_dir  = "./resource/icons";

// -----------------------------------------------------------------------------

Mesh* g_mesh = 0;


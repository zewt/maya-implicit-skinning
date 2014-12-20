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
#include "shader.hpp"
#include "macros.hpp"
#include "mesh.hpp"
#include "vbo_primitives.hpp"
#include "gl_mesh.hpp"
#include "point_cache_export.hpp"

/** @file globals.hpp
    @brief Various global variables
*/

// TODO: encapsulate in namespace when necessary, and provice init() clean() methods
// also add hpp to export specifis variables or group

#include "blending_env_type.hpp"
extern Blending_env::Op_id CUSTOM_TEST;

// -----------------------------------------------------------------------------

extern Point_cache_file* g_anim_cache;

// -----------------------------------------------------------------------------

// please use g_scene_tree.hpp
//#include "scene_tree.hpp"
//extern Scene_tree* g_scene_tree;

// -----------------------------------------------------------------------------
/// Some basic primitive are available, there are upload into the GPU
/// at the initialization of opengl
/// @see init_opengl()
/// @note 'lr' stands for low res
extern VBO_primitives g_primitive_printer;
extern Prim_id g_sphere_lr_vbo;
extern Prim_id g_sphere_vbo;
extern Prim_id g_circle_vbo;
extern Prim_id g_arc_circle_vbo;
extern Prim_id g_circle_lr_vbo;
extern Prim_id g_arc_circle_lr_vbo;
extern Prim_id g_grid_vbo;
extern Prim_id g_cylinder_vbo;
extern Prim_id g_cylinder_cage_vbo;
extern Prim_id g_cube_vbo;


/// Vbo id of the quad used to display the pbo that contains the result
/// of the raytracing done by cuda
extern GLuint g_gl_quad;
// -----------------------------------------------------------------------------


/// Textures used to display controller and operator frames
extern GLuint g_ctrl_frame_tex;
extern GLuint g_op_frame_tex;
extern GLuint g_op_tex;

/// use these to index pbos & textures:
enum{ COLOR = 0,
      DEPTH,
      MAP,
      NORMAL_MAP, ///< scene normals
      NB_TEX      // Keep that at the end
     };

/// Textures that store color and depth buffer + map
extern GLuint g_gl_Tex[NB_TEX];

// -----------------------------------------------------------------------------

/// Various shader programs
extern Shader_prog* g_dummy_quad_shader;
extern Shader_prog* g_lavalamp_program;
extern Shader_prog* g_lavalamp_body_program;
extern Shader_prog* g_arrows_program;
extern Shader_prog* g_points_shader;
extern Shader_prog* g_normal_map_shader;
extern Shader_prog* g_ssao_shader;

/// 'phong_list' is a list of shaders generated from a single file source
/// the enum field specifies which type of shader is in the list
enum {
    NO_TEX,            ///< phong shading no texture
    MAP_KD,            ///< phong shading only diffuse texture
    MAP_KD_BUMP,       ///< phong shading only bump and diffuse textures
    MAP_KD_KS,         ///< phong shading only diffuse and specular textures
    NB_PHONG_SHADERS
};

/// @namespace Tex_units
/// @brief specifies which textures units are used in 'phong_list'
namespace Tex_units{
    extern const int KD;
    extern const int KS;
    extern const int BUMP;
}

extern Shader_prog* g_phong_list[NB_PHONG_SHADERS];

// -----------------------------------------------------------------------------

/// Path to store various caches (mostly blending operators)
extern std::string g_cache_dir;
/// Path to store the app configuration
extern std::string g_config_dir;
/// Path to the icons folder
extern std::string g_icons_dir;

// -----------------------------------------------------------------------------

/// The current mesh
extern Mesh* g_mesh;

// -----------------------------------------------------------------------------

#endif // GLOBAL_HPP__

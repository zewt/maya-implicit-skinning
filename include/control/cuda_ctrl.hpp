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
#ifndef CUDA_CTRL_HPP_
#define CUDA_CTRL_HPP_

#include <string>

#include "constants.hpp"
#include "cuda_rendering.hpp"
#include "blending_env_tex_interface.hpp"
#include "opengl_stuff.hpp"

#include "potential_plane_ctrl.hpp"
#include "animated_mesh_ctrl.hpp"
#include "operators_ctrl.hpp"
#include "skeleton_ctrl.hpp"
#include "display_ctrl.hpp"
#include "debug_ctrl.hpp"
#include "graph_ctrl.hpp"
#include "color_ctrl.hpp"
#include "path_ctrl.hpp"

/** @brief Mouse, keyboard, screen interface and more for the cuda library

  This header provide a control interface for the GUI.
  Because it is always a good thing to separate the GUI from the model we
  provide an interface to control everything related to the cuda lib. Morever
  because GUI is compiled on CPU this also separates nvcc code from
  gcc/visual code
*/

// =============================================================================
namespace Cuda_ctrl{
// =============================================================================

/// Control for the current animated mesh (mesh color, smoothing, anim etc.)
extern Animated_mesh_ctrl* _anim_mesh;
/// Control for the skeleton
extern Skeleton_ctrl      _skeleton;
/// Control display settings (projection ortho/perspective, resolution etc.)
extern Display_ctrl       _display;
/// Control for the debug mode
extern Debug_ctrl         _debug;
/// Control for the current graph (save it load it etc.)
extern Graph_ctrl         _graph;
/// Control for blending operators (bulge in contact, clean union etc.)
extern Operators_ctrl    _operators;
/// List of colors used to display the model, the points, mesh wires etc.
extern Color_ctrl         _color;
/// List of paths used in the application
extern Path_ctrl          _paths;
/// Control potential plane position
extern Potential_plane_ctrl _potential_plane;

// -----------------------------------------------------------------------------

void load_mesh(const std::string file_name);

void load_mesh( Mesh* mesh );

bool is_mesh_loaded();

bool is_animesh_loaded();

bool is_skeleton_loaded();

void erase_graph();

void load_animesh();

void load_animesh_and_ssd_weights(const char* filename);

/// Re compile all the shaders
void reload_shaders();

/// device memory usage in megabytes
void get_mem_usage(double& total, double& free);

// -----------------------------------------------------------------------------

GLuint init_tex_operator( int width, int height );

void init_opengl_cuda();

/// Initialization of the cuda implicit skinning library
/// init glew, opengl, cuda context and pre-compute some heavy stuff
/// such as ultimate blending operators.
/// Sets atexit() so the memory is freed when exit(0); primitive is called
void cuda_start(const std::vector<Blending_env::Op_t>& op);

/// Free CUDA memory
void cleanup();

}// END CUDA_CTRL NAMESPACE ====================================================

// TODO: to be put in the Cuda_ctrl namespace
/// @return wether rendering is complete or not. When false this means
/// display_loop() needs to be called again to complete the rendering. This is
/// because raytracing can be done progressively and not all pixels are computed
/// at the same time. The intermediate results of raytracing are always blurred.
extern bool display_loop(Render_context_cu* ctx);


#endif // CUDA_CTRL_HPP_

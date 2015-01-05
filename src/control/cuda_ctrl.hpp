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

#include "operators_ctrl.hpp"
#include "blending_env_type.hpp"
#include "debug_ctrl.hpp"

class Mesh;

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

/// Control for the debug mode
extern Debug_ctrl         _debug;
/// Control for blending operators (bulge in contact, clean union etc.)
extern Operators_ctrl    _operators;

/// Initialization of the cuda implicit skinning library
/// init glew, opengl, cuda context and pre-compute some heavy stuff
/// such as ultimate blending operators.
/// Sets atexit() so the memory is freed when exit(0); primitive is called
void cuda_start(const std::vector<Blending_env::Op_t>& op);

/// Free CUDA memory
void cleanup();

}// END CUDA_CTRL NAMESPACE ====================================================

#endif // CUDA_CTRL_HPP_

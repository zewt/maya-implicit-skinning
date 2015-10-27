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
void cuda_start(const std::vector<Blending_env::Op_t>& op);

/// Free CUDA memory
void cleanup();

}// END CUDA_CTRL NAMESPACE ====================================================

#endif // CUDA_CTRL_HPP_

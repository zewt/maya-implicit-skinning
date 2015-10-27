#ifndef CONSTANT_TEX_HPP_
#define CONSTANT_TEX_HPP_

#include "constants.hpp"

/** This file MUST be included in the main cuda program file due to the use of
    textures
 */

// =============================================================================
namespace Constants {
// =============================================================================

/// Texture to the list of float constants
extern texture<float, 1,  cudaReadModeElementType> constants_tex;


/// Bind the constants to a texture
void bind();

/// Unbind the constants
void unbind();

}// END NAMSPACE Constants =====================================================

#include "constants_tex.inl"

#endif // CONSTANT_TEX_HPP_

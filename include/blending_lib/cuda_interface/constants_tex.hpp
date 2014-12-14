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
texture<float, 1,  cudaReadModeElementType> constants_tex;


/// Bind the constants to a texture
void bind();

/// Unbind the constants
void unbind();

}// END NAMSPACE Constants =====================================================

#include "constants_tex.inl"

#endif // CONSTANT_TEX_HPP_

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
#ifndef TEX_LOADER_HPP__
#define TEX_LOADER_HPP__

#include "gltex2D.hpp"
#include <string>

/**
  @namespace Tex_loader
  @brief Loading/writting openGL textures utilities

*/
// =============================================================================
namespace Tex_loader{
// =============================================================================

/// @param file_path : path to the texture image. We use Qt to parse the image
/// file, so what QImage can open this function can too.
/// @return An openGL textures
GlTex2D* load(const std::string& file_path);

}
// END TEX_LOADER NAMESPACE ====================================================


#endif // TEX_LOADER_HPP__

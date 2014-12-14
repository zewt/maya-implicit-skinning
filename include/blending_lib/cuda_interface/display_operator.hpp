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
#ifndef DISPLAY_OPERATOR_HPP__
#define DISPLAY_OPERATOR_HPP__

#include "blending_env_type.hpp"

/// Create a texture that samples a blending operator for vizualisation.
/// @param width, height : 2D size of the generated texture
/// @param angle : opening angle ( the scalar product cos(theta) ) you
/// want to display the gradient based operator
/// @return newly allocated operator texture array in host memory. Pixels
/// (x,y) are stored linearly : x + y*width
float* compute_tex_operator(int width, int height,
                            Blending_env::Op_t type,
                            Blending_env::Op_mode mode,
                            float angle, 
                            Blending_env::Op_id custom_id);

#endif // DISPLAY_OPERATOR_HPP__

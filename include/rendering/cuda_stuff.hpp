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
#ifndef CUDA_STUFF_HPP_
#define CUDA_STUFF_HPP_

#include "camera.hpp"

// =============================================================================
namespace Raytracing {
// =============================================================================

/// raytrace the current implicit model
/// @param d_rendu Intermediate buffer used to raytrace
/// @param progressive Activate progressive raytracing : only some pixels are
/// computed at each call. The result is blurred
/// @return if the raytracing is complete (happens when progressive mode is
/// activated)
bool raytrace_implicit(const Camera& cam,
                       float4* d_rendu,
                       float* d_rendu_depth,
                       int* d_img_buf,
                       unsigned* d_depth_buf,
                       int width,
                       int height,
                       bool progressive
                       );

} // END RAYTRACING ============================================================

/// Draw the controller given the last selected bone.
/// Controllers values are directly fetched from texture via a cuda kernel
/// (x, y) the offset to draw the ctrl. (w, h) the width and height
/// @param inst_id instance of the controller
void draw_controller(int inst_id, int x, int y, int w, int h);


#endif // CUDA_STUFF_HPP_

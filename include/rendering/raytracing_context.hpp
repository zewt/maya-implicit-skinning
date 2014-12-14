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
#ifndef RAYTRACING_CONTEXT_HPP__
#define RAYTRACING_CONTEXT_HPP__

#include "packed_data_struct.hpp"
#include "material_cu.hpp"
#include "color.hpp"
#include "render_context.hpp"

// =============================================================================
namespace Raytracing {
// =============================================================================

struct Potential_colors{
    Color negative_color;   /// color for negative potential : f < 0
    Color extern_color;     /// color for external potential : 0 < f < 0.5
    Color intern_color;     /// color for internal potential : 0.5 < f < 1
    Color huge_color;       /// color for huge potential     : 1 < f

//    IF_CUDA_DEVICE_HOST
//    Potential_colors() {
//    }

//    IF_CUDA_DEVICE_HOST
//    ~Potential_colors(){
//    }
};

struct Context {
    /// Grid size to lauch the raytracing kernel
    dim3 grid;
    /// Block size to lauch the raytracing kernel
    dim3 block;

    /// Device buffer to store depth and color resulting from raytracing
    PBO_data pbo;
    /// Uniform material to use for the raytraced scene
    Material_cu mat;
    /// Camera settings for the raytracing
    Camera_data cam;
    /// Draw the 2D slice of the iso-surface
    bool potential_2d;
    /// Draw the scene
    bool draw_tree;

    /// Position of the 2D slice of the iso-surface
    /// @{
    Vec3_cu plane_n;
    Point_cu plane_org;
    /// @}

    /// Enable disable lighting
    bool enable_lighting;
    /// Use environment map
    bool enable_env_map;
    /// Background color if no environment map
    Color background;
    /// defines which pixels are to be drawn in progressive mode
    int3 steps;


    /// Length of a step for the ray marching
    float step_len;
    /// Maximum level of reflexion rays (zero for no reflexions)
    int nb_reflexion;


    Potential_colors potential_colors;

    Render_context* render_ctx;

    Context() : render_ctx(0) { }

};

}// END RAYTRACING =============================================================

#endif// RAYTRACING_CONTEXT_HPP__

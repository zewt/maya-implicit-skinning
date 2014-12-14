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
#ifndef DISPLAY_CTRL_HPP_
#define DISPLAY_CTRL_HPP_

#include "display_operator.hpp"

/** @brief Display controller
  This class is an utility to control display settings such as screen size,
  raytracing multisampling, use of wireframe view etc. Everything that is
  related to the rendering in the viewport.
*/

class Display_ctrl {
public:

    Display_ctrl():
        _draw_voxels(false),
        _draw_hrbf_samples(false),
        _edit_hrbf_samples(false),
        _ssao(false),
        _grid(false),
        _potential_2d(false),
        _oriented_bbox(false),
        _aa_bbox(false),
        _nb_samples_res(128),
        _progressive_raytracing(true),
        _bloom(false),
        _raytracing_lighting(true),
        _raytrace_primitives(true),
        _ray_marching_step_length(0.4f),
        _wire(true),
        _junction_spheres(false),
        _raytrace_again(false),
        _operator_type(Blending_env::MAX),
        _operator_mode(Blending_env::UNION),
        _custom_op_id(-1),
        _opening_angle(0.5f),
        _transparency(0.5f),
        _env_map(false)
    {
    }

    void set_transparency_factor(float factor)
    {
        _transparency = std::min(std::max(factor, 0.f), 1.f);
    }

    bool _draw_voxels;

    bool _draw_hrbf_samples;
    bool _edit_hrbf_samples;

    /// activate screen space ambient oclusion (only in phong rendering)
    bool _ssao;

    /// Draw the grid
    bool _grid;

    /// Show the 2D potential plane when raytracing implicit surfaces
    bool _potential_2d;

    bool _oriented_bbox;
    /// Axis aligned bounded box
    bool _aa_bbox;

    /// nb samples required for one pass of raytracing.
    /// When raytracing is done in progressive mode only _width/_nb_samples_res
    /// and _height/_nb_samples_res are computed for each raytracing passes
    /// untill all screen pixels are computed.
    int  _nb_samples_res;
    /// When progressive raytracing is activated only few pixels are drawn.
    /// if the scene do not change the remaining pixels are also drawn later
    bool _progressive_raytracing;
    /// Activate the bloom effect as a post processing effect of the raytracing
    bool _bloom;
    /// Activate lighting when raytracing
    bool _raytracing_lighting;
    ///
    bool _raytrace_primitives;

    /// Length of a single step for the ray marching
    float _ray_marching_step_length;

    /// Show mesh wires
    bool _wire;
    /// show the junction spheres used to close the implicit primitives
    bool _junction_spheres;

    bool _raytrace_again; ///< When true Raytrace again and set back to false

    Blending_env::Op_t    _operator_type;   ///< type of the operator to be displayed
    Blending_env::Op_mode _operator_mode;   ///< mode of the operator to be displayed
    Blending_env::Op_id   _custom_op_id;    ///< ...
    float _opening_angle; ///< opening angle used to display the operator

    float _transparency; ///< transparency used to display meshes

    bool _env_map; ///< use the environment map in the raytracing
};

#endif // DISPLAY_CTRL_HPP_

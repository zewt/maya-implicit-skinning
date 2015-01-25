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
#ifndef DEBUG_CTRL_HPP__
#define DEBUG_CTRL_HPP__

#include <vector>

class Debug_ctrl {
public:

    Debug_ctrl() :
        _show_gradient(false),
        _do_selection(false),
        _potential_pit(true),
        _step_length(0.05f),
        _raphson(false),
        _collision_threshold(0.9f),
        _propagation_factor(1.f),
        _smooth1_iter(7),
        _smooth2_iter(1),
        _smooth1_force(1.f),
        _smooth2_force(0.5f),
        _smooth_mesh(true),
        _slope_smooth_weight(2)
    {
    }

    bool _show_gradient;

    bool _do_selection;

    bool _potential_pit;

    float _step_length;
    bool  _raphson;
    float _collision_threshold;
    float _propagation_factor;    ///< How mush do we correct the fitting direction
    float _collision_depth;       ///< How mush a vertex interpenetrate when collifing other primitives

    int _smooth1_iter;
    int _smooth2_iter;
    float _smooth1_force;
    float _smooth2_force;
    bool _smooth_mesh;
    int _slope_smooth_weight;
};

#endif // DEBUG_CTRL_HPP__

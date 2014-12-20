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
#ifndef SELECTTION_HEURISTIC_HPP__
#define SELECTTION_HEURISTIC_HPP__

#include <vector>
#include <limits>
#include "port_glew.h"
#include "vec3_cu.hpp"


/** @class Select_type
  @brief mother class of the for selection heuristics

  When points of a mesh are selected various heuristic can be used. For instance
  we can choose the nearest point from the mouse cursor or every points inside a
  circle enterd about the cursor. This class provide an interface to handle
  different heuristics.

  @tparam ID_t the type of the identifier for a point. When a point is selected
  it is this identifier which is stored.
*/

// TODO: the template is not ideal to handle things we should nor store the
// identifier of the points.

template<typename ID_t>
class Select_type {
public:

    Select_type() :
        _z_eps(0.001f)
    {
        _selection.reserve(255);
    }

    /// Erase the selection
    virtual void reset(){ _selection.clear(); }

    /// Test a point for selection given a point and mouse position.
    /// If selected the point 'id' is added to the selection list '_selection'
    /// @see get_selected_points() nb_selected()
    virtual void test(ID_t id, const Vec3_cu& point, const Vec3_cu& mouse) = 0;

    /// Get the list of selected points or NULL if there is no selected point
    virtual ID_t* get_selected_points() {
        return _selection.size() ? &(_selection[0]) : 0;
    }

    virtual int nb_selected() { return _selection.size(); }

protected:
    /// list of selected points
    std::vector<ID_t> _selection;
    /// epsilon used to discard points with a depth higher than the z buffer
    float _z_eps;
};
// =============================================================================


#endif // SELECTTION_HEURISTIC_HPP__

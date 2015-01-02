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
#ifndef LOADER_SKEL_HPP__
#define LOADER_SKEL_HPP__

#include <string>
#include <vector>
#include <transfo.hpp>

// =============================================================================
namespace Loader {

/// @brief intermediate representation of a bone for file loading
struct Abs_bone {
    float       _length; ///< The bone length
    Transfo     _frame;  ///< The bone position and orientation
    std::string _name;   ///< The bone name
};

//------------------------------------------------------------------------------

/// @brief intermediate representation of a skeleton for file loading
struct Abs_skeleton {
    /// Compute the bone lengths of every bones.
    /// The hierachy of the skeleton and bones positions must be correctly filled.
    /// The bone length equal the mean length between the joint and its sons.
    /// Leaves are of length zero
    void compute_bone_lengths();

    /// List of bones
    std::vector<Abs_bone> _bones;
    /// _sons[bone_id] == vec_sons
    std::vector< std::vector<int> > _sons;
    /// _parents[bone_id] == parent_bone_id
    std::vector<int> _parents;
};

}

#endif // LOADER_SKEL_HPP__

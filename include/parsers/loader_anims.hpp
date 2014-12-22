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
#ifndef LOADER_ANIMS_HPP__
#define LOADER_ANIMS_HPP__

#include <vector>
#include "transfo.hpp"

// =============================================================================
namespace Loader {
// =============================================================================

/// @class Base_anim_eval
/// @brief Abstract class to evaluate a skeleton animation
class Base_anim_eval {
public:
    Base_anim_eval(const std::string& name) :
        _name( name ),
        _frame_rate(30.f)
    { }

    virtual ~Base_anim_eval(){ }

    /// @return the local frame of the ith bone for the ith frame
    virtual size_t nb_frames () const = 0;

    /// frame rate in seconds
    float frame_rate() const { return _frame_rate; }

    std::string _name;
    float _frame_rate;
};

//------------------------------------------------------------------------------

/// @class Sampled_anim_eval
/// @brief Implementation of animation evaluator based on matrix samples
/// For each frame and each bone this class stores the associated matrix
class Sampled_anim_eval : public  Base_anim_eval {
public:
    Sampled_anim_eval(const std::string& name) :
        Base_anim_eval( name )
    { }


    size_t nb_frames () const { return _lcl_frames.size();}

    /// Stores every bones local transformations for each frame
    /// _gl_frames[ith_frame][bone_id] == local_transformation
    std::vector< std::vector<Transfo> > _lcl_frames;
};


} // END namespace Loader ======================================================

#endif // LOADER_ANIMS_HPP__

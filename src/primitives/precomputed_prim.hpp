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
#ifndef PRECOMPUTED_PRIM_HPP__
#define PRECOMPUTED_PRIM_HPP__

#include <cassert>
#include "vec3_cu.hpp"
#include "point_cu.hpp"

#include "precomputed_prim_env.hpp"

namespace Skeleton_env {
    typedef int Skel_id;
}

/**
 @class Precomputed_prim
 @brief implicit primitives stored in a 3d grid
*/
struct Precomputed_prim {
    __device__ __host__
    Precomputed_prim() : _id(-1)
    {  }

    void initialize(){
        assert(_id < 0);
        _id = Precomputed_env::new_instance();
    }

    void clear(){
        assert(_id >= 0);
        Precomputed_env::delete_instance(_id);
    }
    __host__
    inline void fill_grid_with(Skeleton_env::Skel_id skel_id, const Bone* bone)
    {
        Precomputed_env::init_instance(_id, skel_id, bone);
    }

    __device__ inline void use_instance_device(int id);

    /// @name Evaluation of the potential and gradient
    /// @{
    __device__
    float f(const Point_cu& p) const;
    __device__
    Vec3_cu gf(const Point_cu& p) const;
    __device__
    float fngf (Vec3_cu& gf, const Point_cu& p) const;
    /// @}

    __host__ inline int get_id() const { return _id; }

private:

    /*-----------*
    | Attributes |
    *-----------*/

    int _id;
};

// -----------------------------------------------------------------------------


//#include "precomputed_prim.inl"

#endif // PRECOMPUTED_PRIM_HPP__


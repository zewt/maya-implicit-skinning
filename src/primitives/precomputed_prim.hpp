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
#include "cuda_utils.hpp"
#include "transfo.hpp"

namespace Skeleton_env {
    typedef int Skel_id;
}
class Bone;

// TODO: we should be able to store any kind of implicit prim (template or class polyphormism)
/** @namespace Precomputed_env
    @brief Environment storing 3D grids representing implicit primitives

    Precomputed_Env provides a way to store in Cuda textures 3d grids and
    fetch them with trilinear interpolation

    How to upload one primitive and transform it:
    @code
    Precomputed_prim inst;
    // init the 3d grid potential and gradient with a implicit bone
    Precomputed_prim.initialize();
    Precomputed_prim.fill_grid_with(skel_id, bone);
    // Apply a global transformation to the 3d grid
    prim.set_transform(tr);
    prim.set_transform(tr);
    // To take into account the transformations
    Precomputed_prim::update_device_transformations();
    @endcode
*/




/**
 @class Precomputed_prim
 @brief implicit primitives stored in a 3d grid
*/
struct PrecomputedInfo;
struct Precomputed_prim {
    IF_CUDA_DEVICE_HOST
    Precomputed_prim() : _id(-1)
    { }

    void initialize();
    void clear();

    __host__
    void fill_grid_with(Skeleton_env::Skel_id skel_id, const Bone* bone);

    /// In order to animate the precomputed primitives one as to set the
    /// transformations applied to each primitive.
    /// @warning One must call update_device_transformations() setting all the
    /// instances transformation
    /// @see update_device_transformations()
    void set_transform(const Transfo& transfo);

    // get the transformation t set by 'set_transform()'
    const Transfo& get_user_transform() const;

    /// Copy the host transformations set by set_transform() to texture
    /// @see set_transform()
    static void update_device_transformations();

#if !defined(NO_CUDA)
    /// @name Evaluation of the potential and gradient
    /// @{
    __device__
    float f(const Point_cu& p) const;
    __device__
    Vec3_cu gf(const Point_cu& p) const;
    __device__
    float fngf (Vec3_cu& gf, const Point_cu& p) const;
    /// @}
#endif

private:
    static void update_device(int _id);
    IF_CUDA_DEVICE_HOST PrecomputedInfo &get_info();
    IF_CUDA_DEVICE_HOST const PrecomputedInfo &get_info() const;

    int _id;
};

// -----------------------------------------------------------------------------


//#include "precomputed_prim.inl"

#endif // PRECOMPUTED_PRIM_HPP__


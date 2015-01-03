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



namespace Skeleton_env {
    typedef int Skel_id;
}
class Bone;


#include "cuda_utils.hpp"
#include "transfo.hpp"

class Bone;
namespace Skeleton_env {
    typedef int Skel_id;
}

// FIXME: we should pad the 3d grid to avoid the lerp artifacts at borders
// TODO: we should be able to store any kind of implicit prim (template or class polyphormism)
/** @namespace Precomputed_env
    @brief Environment storing 3D grids representing implicit primitives

    Precomputed_Env provides a way to store in Cuda textures 3d grids and
    fetch them with trilinear interpolation

    How to upload one primitive and transform it:
    @code
    int id = new_instance();
    // init the 3d grid potential and gradient with a implicit bone
    init_instance(id, bone);
    // Apply a global transformation to the 3d grid
    set_transform(id, tr);
    set_transform(id, tr);
    // To take into account the transformations
    update_device_transformations();
    @endcode
*/
// =============================================================================
namespace Precomputed_env{
// =============================================================================

// -----------------------------------------------------------------------------

void clean_env();

/// get the transformation t set by 'set_transform()'
const Transfo& get_user_transform(int inst_id);

/// In order to animate the precomputed primitives one as to set the
/// transformations applied to each primitive.
/// @warning One must call update_device_transformations() setting all the
/// instances transformation
/// @see update_device_transformations()
void set_transform(int inst_id, const Transfo& transfo);

/// Copy the host transformations set by set_transform() to texture
/// @see set_transform()
void update_device_transformations();

}// END PRECOMPUTED_ENV NAMESPACE ==============================================






/**
 @class Precomputed_prim
 @brief implicit primitives stored in a 3d grid
*/
struct Precomputed_prim {
    IF_CUDA_DEVICE_HOST
    Precomputed_prim() : _id(-1)
    {  }

    void initialize();
    void clear();

    __host__
    void fill_grid_with(Skeleton_env::Skel_id skel_id, const Bone* bone);

#if !defined(NO_CUDA)
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
#endif

    __host__ inline int get_id() const { return _id; }

private:
    
public: // XXX
    static void dealloc(int _id);
    static void update_info(int _id);

    int _id;
};

// -----------------------------------------------------------------------------


//#include "precomputed_prim.inl"

#endif // PRECOMPUTED_PRIM_HPP__


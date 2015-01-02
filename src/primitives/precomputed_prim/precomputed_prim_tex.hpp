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
#ifndef PRECOMPUTED_PRIM_TEX_HPP__
#define PRECOMPUTED_PRIM_TEX_HPP__

#include "point_cu.hpp"
#include "precomputed_prim_env.hpp"

/**
    @file precomputed_prim_tex.hpp
    This files handles the textures that store the precomputed values of
    implicit primitives in 3d grids.

    <b>There is some restrictions with the use of cuda texture read carrefully
    what follows to use this header correctly </b>

    Cuda textures scope is limited to a single '.cu'. Inclusion of this header
    in several '.cu' will create DIFFERENT texture references inside
    each '.cu'. As a result in EACH '.cu' you MUST bind the textures before
    using them in a kernel. You can do it with 'bind_local()'. Don't forget to
    unbind with unbind_local().

    Sometimes you want to remotly bind/unbind textures inside a '.cu'.
    You can't use bind_local() for this you must use bind() in the header
    ****_env_tex_binding.hpp; This header can be included only once in the
    entire project. Moreover it must be included just after this header.
    Therefore remotly bind/unbind texture can be done only for a single .cu
    in the project.
 */

// =============================================================================
namespace Precomputed_env{
// =============================================================================

/// First float is the potential last three floats the gradient
extern texture<float4, 3, cudaReadModeElementType> tex_grids;
extern texture<float4, 1, cudaReadModeElementType> tex_transform;
extern texture<float4, 1, cudaReadModeElementType> tex_transform_grad;
extern texture<int4  , 1, cudaReadModeElementType> tex_offset_;

// -----------------------------------------------------------------------------

static void bind_local();

static void unbind_local();

__device__
float fetch_potential_and_gradient(int inst_id, const Point_cu& p, Vec3_cu& grad);

__device__
float fetch_potential(int inst_id, const Point_cu& p);

__device__
Vec3_cu fetch_gradient(int inst_id, const Point_cu& p);

}
// END PRECOMPUTED_ENV NAMESPACE ===============================================

#include "precomputed_prim_tex.inl"

#endif // PRECOMPUTED_PRIM_TEX_HPP__

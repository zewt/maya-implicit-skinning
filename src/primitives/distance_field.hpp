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
#ifndef DISTANCE_FIELD_HPP__
#define DISTANCE_FIELD_HPP__

#include "cuda_compiler_interop.hpp"
#include "macros.hpp"
#include "vec3_cu.hpp"

/**
 * @namespace Field
 * @brief tools to manipulate/transform/scale potential field of implicit objects
*/
// =============================================================================
namespace Field{
// =============================================================================

    IF_CUDA_DEVICE_HOST
    inline float distance_squared_to_field(float d2, float radius);

    IF_CUDA_DEVICE_HOST
    inline float distance_to_field(float d, float radius);

    IF_CUDA_DEVICE_HOST
    inline float field_derivative_from_distance(float d, float radius);

    IF_CUDA_DEVICE_HOST
    inline float2 distance_to_field_and_derivative(float d, float radius);

    IF_CUDA_DEVICE_HOST
    inline float field_to_distance(float f, float radius);

    ///////////////////////////
    IF_CUDA_DEVICE_HOST
    inline float distance_to_field_flatten(float d, float r, float offset);
    IF_CUDA_DEVICE_HOST
    inline float field_derivative_from_distance_flatten(float d, float r, float offset);
    IF_CUDA_DEVICE_HOST
    inline float2 distance_to_field_and_derivative_flatten(float d, float r, float offset);
    ///////////////////////////

    /// Transform a global distance field to a compact field of radius radius
    /// if( -radius < f < radius) to_compact(f) = (1/4) * (f/radius)^3 - (3/4) * (f/radius) + 1/2
    ///	if( f >  radius)          to_compact(f) = 0
    /// if( f < -radius)          to_compact(f) = 1
    /// junction at radius and -radius is C1 with
    IF_CUDA_DEVICE_HOST
    inline float to_compact_poly_c1(float f, float radius);

    /// Transform the gradient with the C1 poly
    IF_CUDA_DEVICE_HOST
    inline void grad_to_compact_poly_c1(float f, float radius, Vec3_cu& grad);

    /// to_compact(f) = (-3/16)*(f/radius)^5 + (5/8)*(f/radius)^3 - (15/16)*(f/radius) + 1/2
    /// junction is C2 @see to_compact_poly_c2
    IF_CUDA_DEVICE_HOST
    inline float to_compact_poly_c2(float f, float radius);

    /// Transform the gradient with the C2 poly
    IF_CUDA_DEVICE_HOST
    inline void grad_to_compact_poly_c2(float f, float radius, Vec3_cu& grad);

    /// to_compact(f) = exp(f*(-to/radius)) / (exp(f*(-to/radius))+1)
    /// when 'to' is inf function is Cinf
    /// to = 5 or 7 is advised
    IF_CUDA_DEVICE_HOST
    inline float to_compact_tanh(float f, float radius, float to);

    /// Transform the gradient with the Cinf tanh
    IF_CUDA_DEVICE_HOST
    inline void grad_to_compact_tanh(float f, float r, float to, Vec3_cu& grad);

}// End FIELD ==================================================================

#include "distance_field.inl"

#endif // DISTANCE_FIELD_HPP__

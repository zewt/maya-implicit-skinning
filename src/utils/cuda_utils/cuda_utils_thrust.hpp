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
#ifndef CUDA_UTILS_THRUST_HPP__
#define CUDA_UTILS_THRUST_HPP__

#include <thrust/scan.h>
#include <thrust/device_ptr.h>
#include <cassert>

#include "cuda_utils.hpp"

/**

  @file cuda_utils_thrust.hpp
  @brief interoperability between cuda utils and thrust

  To prevent re-inventing the wheel we use thrust to launch standard parallel
  algorithms on device arrays


*/

// =============================================================================
namespace Cuda_utils{
// =============================================================================

/// inclusive scan is a prefix sum which skips the first element of the array
/// Computation is done in place. A prefix sum consist in summing every elements
/// before the element.
/// example:
/// array input  : 2 1 1 2 3 -1
/// array output : 2 3 4 6 9  8
/// @param start first index to treat
/// @param end : last index to treat
template<class T>
void inclusive_scan(int start,
                    int end,
                    Cuda_utils::Device::Array<T>& array)
{
    assert(start >= 0           );
    assert(end   <  array.size());
    thrust::device_ptr<T> d_ptr = thrust::device_pointer_cast( array.ptr() );

    thrust::inclusive_scan(d_ptr+start, d_ptr+end+1, d_ptr);
}

// -----------------------------------------------------------------------------

template<class T>
void inclusive_scan(int start, int end, T* array)
{
    thrust::device_ptr<T> d_ptr = thrust::device_pointer_cast( array );
    thrust::inclusive_scan(d_ptr+start, d_ptr+end+1, d_ptr);
}

// -----------------------------------------------------------------------------

template<class T>
void pack(Cuda_utils::Device::Array<T>& array)
{

}

}// END Cuda_utils =============================================================

#endif // CUDA_UTILS_THRUST_HPP__

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
#ifndef DEVICE_ELT_HPP__
#define DEVICE_ELT_HPP__

//#include "cuda_utils_common.hpp"
#include "macros.hpp"
#include <vector>
#include <iostream>

// =============================================================================
namespace Cuda_utils{
// =============================================================================

// =============================================================================
namespace Device{
// =============================================================================

/// @class Elt
/// @brief Automatic upload of an element of type 'T' to GPU
template <class T>
struct Elt {

    Elt(const T& elt) {
        malloc_d( _d_ptr, 1);
        mem_cpy_htd(_d_ptr, elt, 1);
    }

    ~Elt() { free_d(_d_ptr); }

    T* ptr(){ return _d_ptr; }

private:
    T* _d_ptr;
};

}// END Device =================================================================

}// END Cuda_utils =============================================================

#endif //DEVICE_ELT_HPP__

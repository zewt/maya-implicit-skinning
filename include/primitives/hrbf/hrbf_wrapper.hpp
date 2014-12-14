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
#ifndef HRBF_WRAPPER_HPP__
#define HRBF_WRAPPER_HPP__

#include "hrbf_phi_funcs.hpp"
#include "hrbf_data.hpp"
#include "hrbf_setup.hpp"

/** @brief Wrapper interface of the RBF's classes
    The wrapper is design to separate nvcc code to gcc code.

    The below functions are to be called from __host__ cuda functions.
    The wrapper avoid nvcc compilation of the RBF's classes code which involve
    unsuported C++ feature. RBF's classes code are to be compile with gcc
    (i.e the file RBF.h and rbf_wrapper.cpp).

    How to use it :
    @code

    \endcode
*/

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

/// Compute Hermite RBF coeffs with the given points and normals
/// @param res : result of the fit with the computed coeficients
void hermite_fit(const Vec3_cu* points,
                 const Vec3_cu* normals,
                 int size,
                 HRBF_coeffs& res);

}// END RBF_WRAPPER ============================================================

#endif // HRBF_WRAPPER_HPP__

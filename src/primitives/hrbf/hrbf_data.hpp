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
#ifndef HRBF_DATA_HPP
#define HRBF_DATA_HPP

#include "vec3_cu.hpp"

/**
    Data structure to communicate with RBF between nvcc and gcc.
    The RBF wrapper has to utilize data structure which is both compatible
    with gcc and nvcc.
*/

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

    /// HermiteRbfReconstruction data wrapper in order to store RBF coeffs
    ///	for post evaluation of the potential field
    struct HRBF_coeffs {

        float*   alphas;
        Vec3_cu* nodeCenters;
        Vec3_cu* normals;
        Vec3_cu* betas;
        int size;        ///< size of the previous arrays

        ~HRBF_coeffs (){
            delete[] alphas;
            delete[] nodeCenters;
            delete[] normals;
            delete[] betas;
        }
    };

}// END RBF_wrapper ============================================================

#endif // HRBF_DATA_HPP

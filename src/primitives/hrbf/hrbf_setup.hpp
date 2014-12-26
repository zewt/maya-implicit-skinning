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
#ifndef HRBF_SETUP_HPP_
#define HRBF_SETUP_HPP_

#include "hrbf_phi_funcs.hpp"

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

   const int   RBF_POLY_DEG = 1;
   const float MESH_SIZE    = 15.0f;

   // thin plates

#define HERMITE_WITH_X3 1
//#define HERMITE_WITH_THIN_PLATES 1

   // Change type in order to use another phi_function from rbf_phi_funcs.hpp :
#if defined(HERMITE_WITH_X3)
   typedef Rbf_pow3<float> PHI_TYPE;
#elif defined(HERMITE_WITH_THIN_PLATES)
   typedef Rbf_thin_plate<float> PHI_TYPE;
   //typedef Rbf_x_sqrt_x<float> PHI_TYPE;
#endif

}// END RBf_wrapper ============================================================

#endif // HRBF_SETUP_HPP_

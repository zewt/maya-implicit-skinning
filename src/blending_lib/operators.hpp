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
#ifndef IBL_OPERATORS_HPP__
#define IBL_OPERATORS_HPP__

#include <string>

#include "structs.hpp"

/**
  @namespace IBL
  @brief Implicit Blending Library (IBL) provides blending operators for
  implicit surfaces


*/
// =============================================================================
namespace IBL {
// =============================================================================

// TODO add capacity to specify which operator to load into memory
void init(const std::string& cache_path);


class Operator {
public:
    Operator(){}

    virtual float eval(IBL::float3& grad_res,
                       float pot0,
                       float pot1,
                       const IBL::float3& grad0,
                       const IBL::float3& grad1) const = 0;

};



/** @namespace IBL::Discreet
    @brief Blending operators evaluated from a precomputed grid with bilinear
    interpolation
*/
// =============================================================================
namespace Discreet {
// =============================================================================



}// END Discreet ===============================================================


/** @namespace IBL::Continuous
    @brief Blending operators that are evaluated with their analytical solutions
*/
// =============================================================================
namespace Continuous {
// =============================================================================



}// End Continuous =============================================================


}// END IBL ====================================================================

#endif // IBL_OPERATORS_HPP__

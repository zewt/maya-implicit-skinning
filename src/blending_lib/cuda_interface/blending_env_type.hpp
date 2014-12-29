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
#ifndef BLENDING_ENV_TYPE_HPP
#define BLENDING_ENV_TYPE_HPP


// =============================================================================
namespace Blending_env {
// =============================================================================

/// Type of a controller's identifier
typedef int Ctrl_id;
/// Type of an operator indentifier
typedef int Op_id;

enum Op_t {
    /// @name Unary operators
    NEG = 0,           ///< Standard negation operator
    /// @name Binary Standard operators
    DIFF,         ///< Standard Difference
    DIFF_SMOOTH,  ///< Smoothed Difference
    /// @name N-ary operators
    MAX,          ///< Standard Union
    MIN,          ///< Standard Union
    SUM,          ///< Standard n-ary blend
    RICCI,        ///< Ricci 1973's blend
    C_CONTACT,    ///< Cani 1993's contact
    C_CONTACT_UNAIRE,    ///< Cani 1993's contact unary modifier
    W_GRAPH,      ///< Wyvill 1995's composition graph
    W_RESTRICTED, ///< Wyvill 2010's blend
    W_RESTRICTED_UNAIRE, ///< Wyvill 2010's blend unary modifier
    NARY_OPERATOR_END,
    /// @name Binary 4D operators
    B_OH_4D,      ///< 4D with bulge profile controlled by strengh and open hyperbola opening
    R_OH_4D,      ///< 4D with ricci profile controlled by parameter N and open hyperbola opening
    BINARY_4D_OPERATOR_END,
    /// @name Binary 3D operators
    U_RH,         ///< 3D operator with ultimate profile and restricted open hyperbola opening
    BINARY_3D_OPERATOR_BEGIN,
    C_L,          ///< 3D operator with circle profile and line opening
    C_D,          ///< 3D operator with circle profile and diamond opening
    C_OH,         ///< 3D operator with circle profile and open hyperbola opening
    C_HCH,        ///< 3D operator with circle profile and hermite-closed hyperbola opening
    C_TCH,        ///< 3D operator with circle profile and tanh-closed opening
    U_OH,         ///< 3D operator with ultimate profile and open hyperbola opening
    U_HCH,        ///< 3D operator with ultimate profile and hermite-closed hyperbola opening
    U_TCH,        ///< 3D operator with ultimate profile and tanh-closed opening
    B_OH,         ///< 3D operator with bulge profile and open hyperbola opening
    B_HCH,        ///< 3D operator with bulge profile and hermite-closed opening
    B_TCH,        ///< 3D operator with bulge profile and tanh-closed opening
    /////////
    // TODO: add a new switch case in display_operator.cu for B_D operator to be displayed:
    B_D,          ///< 3D operator with bulge profile and diamond opening
    ////////
    BINARY_3D_OPERATOR_END,
    CUSTOM        ///< Means operators is user defined and is to be access with Op_id
};

enum Op_mode {
    UNION = 0,    ///< specifies Union-defined operators
    INTERSECTION, ///< specifies Intersection-defined operators
    DIFFERENCE_    ///< specifies Difference-defined operators
};

}// END BLENDING_ENV NAMESPACE =================================================

#endif // BLENDING_ENV_TYPE_HPP

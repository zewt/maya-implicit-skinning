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
#ifndef OPERATOR_USE_MACROS_HPP
#define OPERATOR_USE_MACROS_HPP

// ############# defines with " // * " are not implemented for use in a CSG Tree.
#define USE_NEGATION
#define USE_MAX_OP
#define USE_MIN_OP
#define USE_SUM_OP
#define USE_RICCI_OP
#define USE_SOFT_DIFF_OP
#define USE_SHARP_DIFF_OP
//#define USE_CANI_OP        // *
//#define USE_GRAPH_OP       // *
//#define USE_RESTRICTED_OP  // *
#define USE_RICCI4D_OP
#define USE_BHO_OP
#define USE_BHC_OP
#define USE_CL_OP
#define USE_CD_OP
#define USE_CHO_OP
#define USE_CHC_OP
#define USE_ICHO_OP
#define USE_ICHC_OP
#define USE_DCHO_OP
#define USE_DCHC_OP
#define USE_UHO_OP
#define USE_UHC_OP
#define USE_IUHO_OP
#define USE_IUHC_OP
#define USE_DUHO_OP
#define USE_DUHC_OP

#define USE_UHR_OP
#define USE_IUHR_OP
#define USE_DUHR_OP

// Had to use a namespace to avoid name conflicts of the enums ...
// FIXME: do the same for OperatorType (is is not a good idea to pollute
// the global namespace especialy with common names like MIN, MAX ...)
// =============================================================================
namespace EPrim {
// =============================================================================

enum PrimitiveType{
    BLOB = 0,
    CYLINDER,
    PLANE,
    HRBF
};

}// END Eprim ==================================================================

#endif // OPERATOR_USE_MACROS_HPP

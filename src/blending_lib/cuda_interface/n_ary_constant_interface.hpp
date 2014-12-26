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
#ifndef N_ARY_CONSTANT_INTERFACE_HPP__
#define N_ARY_CONSTANT_INTERFACE_HPP__

/**
 * @file N_ary_constant_interface
 * @brief Interface to access n_ary.hpp constant parameters
*/

// =============================================================================
namespace N_ary {
// =============================================================================

void set_RICCI_N(float v);

void set_wA0A0(float v);
void set_wA0A1(float v);
void set_wA1A1(float v);
void set_wA1A0(float v);
void set_a0   (float v);
void set_w0   (float v);
void set_a1   (float v);
void set_w1   (float v);
void set_gji  (float v);
void set_gij  (float v);

} // END N_ary =================================================================

#endif // N_ARY_CONSTANT_INTERFACE_HPP__

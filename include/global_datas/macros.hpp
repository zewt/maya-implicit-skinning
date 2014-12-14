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
#ifndef MACRO_HPP_
#define MACRO_HPP_

/// @file macros.hpp @brief various project macros

// Hermite RBF radius for the pseudo compact support
#define HRBF_RADIUS (7.)

/** The power factor in implicit function equations.
    The formula is:
    f = (1 - (distance/radius)^2)^ALPHA
 */
#define ALPHA 4

#define POTENTIAL_ISO (0.5f)
//#define LARGE_SUPPORT_PRIMITIVES

/// The size of a block in 2D CUDA kernels
#define BLOCK_SIZE_X 8
#define BLOCK_SIZE_Y 8

/// Sets the amount of multisampling in X and Y
// FIXME: multisampling only works for x = y = 1 ...
// everything is broken since the adds of progressive mode
#define MULTISAMPX 1
#define MULTISAMPY 1

#endif // MACRO_HPP_

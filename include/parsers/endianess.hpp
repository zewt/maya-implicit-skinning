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
#ifndef ENDIANESS_HPP__
#define ENDIANESS_HPP__

/**
    @namespace Endianess
    @brief System independant conversions between big or little endian.

    These functions are independent from the system endianess as long as
    'init()' is called before using the conversion functions
*/

// =============================================================================
namespace Endianess {
// =============================================================================

/// setup the pointer functions depending on the system endianess.
void init();

/// @return wether the system is big or little endian.
bool is_little_endian();

/// @return a big endian short
extern short (*big_short)    ( short s );
/// @return a little endian short
extern short (*little_short) ( short s );
/// @return a big endian long
extern int   (*big_long)     ( int i   );
/// @return a little endian long
extern int   (*little_long)  ( int i   );
/// @return a big endian float
extern float (*big_float)    ( float f );
/// @return a little endian float
extern float (*little_float) ( float f );

}
// =============================================================================



#endif // ENDIANESS_HPP__

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
/// Portable interface between os and compilers
/// for glew.h header
#ifndef PORT_GLEW_H_
#define PORT_GLEW_H_

#if defined(_WIN32) && defined(_MSC_VER)
    #ifndef NOMINMAX
        #define NOMINMAX // Prevents definition of macro min max in windows.h
    #endif

    #include <windows.h>

    // Undef window.h polluting macros
    #ifdef DIFFERENCE
        #undef DIFFERENCE
    #endif

    #include <GL/glew.h>
#else
    #include <GL/glew.h>
#endif

#endif // PORT_GLEW_H_

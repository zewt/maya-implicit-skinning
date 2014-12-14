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
#ifndef ASSERT_UTILS_HPP__
#define ASSERT_UTILS_HPP__

#include <cassert>

/// @brief asserts and print a message if triggered
#ifndef NDEBUG
#define assert_msg(a, str)                  \
while(0){                                   \
    if(!(a)){                               \
        std::cerr << (str) << std::endl;    \
        assert(false);                      \
    }                                       \
}

#else
#define assert_msg(a, str)
#endif

#endif // ASSERT_UTILS_HPP__

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
#include "tools.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>

// =============================================================================
namespace IBL {
// =============================================================================

double f_inverse(double y,
                 double (*f)(double x),
                 double xmin,
                 double xmax,
                 double eps)
{
    // Check we are within range [xmin, xmax]
    assert( std::min( std::max( f(xmin), f(xmax) ), y ) == y );

    bool rising = f(xmax) > f(xmin);
    double x0 = xmin;
    double x1 = xmax;

    double x_mid = x0;
    int acc = 0;
    while( std::abs(f(x_mid) - y) > eps )
    {
        if( acc++ > 1000) break; // avoid infinite loops

        x_mid = (x0+x1) * 0.5f;
//        const bool s = rising ? f(x_mid) > y : f(x_mid) <= y;
//        if( s ) x1 = x_mid;

        if ( !( (f(x_mid) > y) ^ (rising)) ) x1 = x_mid;
        else                                 x0 = x_mid;
    }

    return x_mid;
}

}// END IBL ====================================================================

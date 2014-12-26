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
#ifndef IBL_TOOLS_HPP__
#define IBL_TOOLS_HPP__


// =============================================================================
namespace IBL {
// =============================================================================

/// Compute numerically the inverse of f within the range [xmin, xmax].
/// f must be monotonic
/// @param y : abscisa of f_inv
/// @param f : function to invert (f must be monotonic)
/// @param xmin, xmax : abscisa range used to perform the dychotomic search to
/// find y.
/// @param eps : threshold of precision to stop the dychotomic search
/// @return the value returned by f_inv( y ) (or the x corresponding to f(x) = y)
/// y must in [f(min), f(max)] otherwise result is undefined.
double f_inverse(double y, double (*f)(double x), double xmin, double xmax, double eps = 1e-5);

}// END IBL ====================================================================

#endif // IBL_TOOLS_HPP__

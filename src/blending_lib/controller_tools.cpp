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
#include "controller_tools.hpp"

#include <cmath>

// =============================================================================
namespace IBL {
// =============================================================================

// -----------------------------------------------------------------------------

float usig(float x){ return 1 - exp(1 - 1/(1-x)); }

float dusig(float x){ return exp(1 - 1/(1-x))/((1-x)*(1-x)); }

float vsig(float x){ return exp(1 - 1/x); }

float dvsig(float x){ return exp(1 - 1/x)/(x*x); }

// -----------------------------------------------------------------------------

float sigpos(float x, float slope1)
{
    if(slope1 >= 1.f){
        return powf(usig(vsig(x*0.8f + 0.1f)),slope1);
    } else {
        slope1 = 2.f - slope1;
        return 1.f - powf(1.f - usig(vsig(x*0.8f + 0.1f)),slope1);
    }
}

// -----------------------------------------------------------------------------

float signeg(float x, float slope0)
{
    if(slope0 >= 1.f){
        return powf(usig(vsig(x*0.8f + 0.1f)),slope0);
    } else {
        slope0 = 2.f - slope0;
        return 1.f - powf(1.f - usig(vsig(x*0.8f + 0.1f)),slope0);
    }
}

// -----------------------------------------------------------------------------

float dsig(float x, float slope)
{
    if(slope >= 1.f){
        return slope*0.8f*dvsig(x*0.8f + 0.1f)*dusig(vsig(x*0.8f + 0.1f))*
                pow(usig(vsig(x*0.8f + 0.1f)),slope-1.f);
    } else {
        slope = 2.f - slope;
        return slope*0.8f*dvsig(x*0.8f + 0.1f)*dusig(vsig(x*0.8f + 0.1f))*
                pow(1.f- usig(vsig(x*0.8f + 0.1f)),slope-1.f);
    }
}

// -----------------------------------------------------------------------------

}
// END IBL =====================================================================

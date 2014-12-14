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
#include "opening.hpp"

#include "tools.hpp"
#include "funcs.hpp"

// =============================================================================
namespace IBL {
// =============================================================================

// =============================================================================
namespace Opening {
// =============================================================================

Base* make(Kind_t opening_type)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    Base* opening = 0;

    switch (opening_type)
    {
    case IBL::Opening::LINE:        opening = new IBL::Opening::Line();    break;
    case IBL::Opening::DIAMOND:     opening = new IBL::Opening::Diamond(); break;
    case IBL::Opening::OPEN_TANH:   opening = new Dh(Dh::OPEN_TANH);       break;
    case IBL::Opening::CLOSED_H:    opening = new Dh(Dh::CLOSED_HERMITE);  break;
    case IBL::Opening::CLOSED_TANH: opening = new Dh(Dh::CLOSED_TANH);     break;
    default:
        std::cerr << "ERROR: openig type doesn't exists" << std::endl;
        break;
    }

    return opening;
}

// -----------------------------------------------------------------------------

bool Pan_hf::_is_init;
float Pan_hf::_vals[];

// -----------------------------------------------------------------------------

void Pan_hf::init_samples()
{
    if(_is_init) return;


    for(int i = 0; i < _nb_samples; i++)
    {
        float t = 2.f * ((float)i / (float)(_nb_samples-1));
        _vals[i] = (float)f_inverse(t, f_hyperbola, 0.f, 1.f);
        //printf("%d- %f; ",i, _vals[i]);
    }
    _vals[_nb_samples-1] = 1.f;
    _is_init = true;
}

// -----------------------------------------------------------------------------


}// END IBL ====================================================================

}// END OPENING ================================================================


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
#include "controller.hpp"

#include <cassert>

#include "controller_tools.hpp"
#include "generator.hpp"

// =============================================================================
namespace IBL {
// =============================================================================

    // =========================================================================
    namespace Shape {
    // =========================================================================

    IBL::Ctrl_setup caml()
    {
        IBL::Ctrl_setup sh(IBL::make_float2(-3.f, 1.f),
                           IBL::make_float2(0.f, 0.f),
                           IBL::make_float2(3.f, 1.f),
                           -4.f, -4.f);
        return sh;
    }

    // -------------------------------------------------------------------------

    IBL::Ctrl_setup finger()
    {
        IBL::Ctrl_setup sh(IBL::make_float2(-1.f, 1.f),
                           IBL::make_float2(0.2f, 0.3f),
                           IBL::make_float2(0.75f, 1.f),
                           1.f, 1.f);
        return sh;
    }

    // -------------------------------------------------------------------------

    IBL::Ctrl_setup elbow()
    {
        IBL::Ctrl_setup sh(IBL::make_float2(0.2f, 1.00f),
                           IBL::make_float2(0.7f, 0.43f),
                           IBL::make_float2(1.2f, 1.00f),
                           1.f, 1.f);
        return sh;
    }

    // -------------------------------------------------------------------------

    IBL::Ctrl_setup flat_up()
    {
        IBL::Ctrl_setup sh(IBL::make_float2(-3.0f, 1.0f),
                           IBL::make_float2( 0.0f, 1.0f),
                           IBL::make_float2( 3.0f, 1.0f),
                           0.f, 0.f);
        return sh;
    }

    // -------------------------------------------------------------------------

    IBL::Ctrl_setup flat_down()
    {
        IBL::Ctrl_setup sh(IBL::make_float2(-3.0f, 0.0f),
                           IBL::make_float2( 0.0f, 0.0f),
                           IBL::make_float2( 3.0f, 0.0f),
                           0.f, 0.f);
        return sh;
    }

    // -------------------------------------------------------------------------

    }
    // END Shape ===============================================================


    // =========================================================================
    namespace Discreet {
    // =========================================================================

    Controller::Controller(const Ctrl_setup& shape, int nb_samples) :
        _shape(shape)
    {
        gen_controller(nb_samples, _shape, _func_vals);
    }

    Controller::~Controller(){ delete[] _func_vals; }

    // -------------------------------------------------------------------------

    float Controller::eval( float dot ) const
    {
        // TODO: to be implemented with linear filtering
        assert(false);
        return 0.f;
    }

    // -------------------------------------------------------------------------

    void Controller::update_shape(const Ctrl_setup& shape, int nb_samples)
    {
        delete[] _func_vals;
        _shape = shape;
        gen_controller(nb_samples, _shape, _func_vals);
    }

    // -------------------------------------------------------------------------

    }
    // END Discreet ============================================================


    // =========================================================================
    namespace Continuous {
    // =========================================================================

    float Controller::eval( float dot ) const
    {
        float b0 = _shape.p0().x;
        float b1 = _shape.p1().x;
        float b2 = _shape.p2().x;
        float F0 = _shape.p0().y;
        float F1 = _shape.p1().y;
        float F2 = _shape.p2().y;
        float S0 = _shape.s0();
        float S1 = _shape.s1();

        float fdot;
        if(dot < b1){
            if(dot < b0){
                fdot = F0;
            } else {
                float a = 1.f/(b0-b1);
                float b = - b1 * a;
                fdot = signeg(a * dot + b, S0);
                if(b0 < -1.f){
                    fdot /= signeg(b - a, S0);
                }
                fdot = (F0 - F1) * fdot  + F1;
            }
        } else {
            if(dot > b2){
                fdot = F2;
            } else {
                float a = 1.f/(b2-b1);
                float b = - b1 * a;
                fdot = sigpos(a * dot + b, S1);
                if(b2 > 1.f)
                    fdot /= sigpos(a + b, S1);
                fdot = (F2 - F1) * fdot + F1;

            }
        }
        return fdot;
    }

    // -------------------------------------------------------------------------

    void Controller::update_shape(const Ctrl_setup& shape)
    {
        _shape = shape;
    }

    // -------------------------------------------------------------------------

    }
    // END Discreet ============================================================

}
// END IBL =====================================================================

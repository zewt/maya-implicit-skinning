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
#ifndef BLENDING_ENV_TEX_INTERFACE_HPP_
#define BLENDING_ENV_TEX_INTERFACE_HPP_


#include "blending_lib/controller.hpp"

/**
    Enable other compilation unit to interact with operator texture when this
    file is included.

    Because texture and kernels using them are to be in the same file scope
    (i.e same compilation unit) this means texture header blending_env_tex.hpp
    cannot be included at several locations because it would create multiple
    declaration of global texture reference.
*/

// =============================================================================
namespace Blending_env{
// =============================================================================

    extern float eval_global_ctrl(float dot);
    extern void set_global_ctrl_shape(const IBL::Ctrl_setup& shape);
    extern IBL::Ctrl_setup get_global_ctrl_shape();
    extern void set_bulge_magnitude(float mag);
    extern void set_ricci_n(float N);

    extern void init();
    extern void unbind();

}// END Blending_env ===========================================================

#endif // BLENDING_ENV_TEX_INTERFACE_HPP_

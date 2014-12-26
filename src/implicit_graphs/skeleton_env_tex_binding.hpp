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
#ifndef SKELETON_ENV_TEX_BINDING_HPP
#define SKELETON_ENV_TEX_BINDING_HPP

/**
  Please refer to the documentation of skeleton_tex.hpp for more details
  on how to use this header

  @warning This file may only be included once in the project
  (i.e. in a single .cu), and ONLY after the inclusion of the header
  skeleton_tex.hpp

  @see skeleton_tex.hpp
*/


#if !defined(SKELETON_ENV_TEX_HPP__)
#error "You must include skeleton_env_tex.hpp before the inclusion of 'skeleton_env_tex_binding.hpp'"
#endif

// =============================================================================
namespace Skeleton_env {
// =============================================================================

bool binded = false;

// -----------------------------------------------------------------------------

void bind()  { bind_local();   binded = true;   }
void unbind(){ unbind_local(); binded = false;  }

// -----------------------------------------------------------------------------

}// END Skeleton_env ===========================================================

#endif // SKELETON_ENV_TEX_BINDING_HPP

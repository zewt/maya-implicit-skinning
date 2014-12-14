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
#ifndef PRECOMPUTED_PRIM_ENV_BINDING_HPP__
#define PRECOMPUTED_PRIM_ENV_BINDING_HPP__

/**
  Please refer to the documentation of precomputed_prim_env_tex.hpp for more
  details on how to use this header

  @warning This file may only be included once in the project
  (i.e. in a single .cu), and ONLY after the inclusion of the header
  precomputed_prim_tex.hpp

  @see precomputed_prim_env_tex.hpp
*/


#if !defined(PRECOMPUTED_PRIM_TEX_HPP__)
#error "You must include precomputed_prim_tex.hpp before the inclusion of 'precomputed_prim_env_binding.hpp'"
#endif

// =============================================================================
namespace Precomputed_env{
// =============================================================================

bool binded = false;

// -----------------------------------------------------------------------------

void bind()  { bind_local();   binded = true;   }
void unbind(){ unbind_local(); binded = false;  }

// -----------------------------------------------------------------------------

}// END Skeleton_env ===========================================================

#endif // PRECOMPUTED_PRIM_ENV_BINDING_HPP__

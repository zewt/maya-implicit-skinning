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
#pragma once
#include "precomputed_prim.hpp"

#if !defined(PRECOMPUTED_PRIM_TEX_HPP__)
#error "You must include precomputed_prim_tex.hpp before the inclusion of 'precomputed_prim.inl'"
#endif

// -----------------------------------------------------------------------------

__device__
inline float Precomputed_prim::f(const Point_cu& x) const
{
    return Precomputed_env::fetch_potential(_id, x);
}

// -----------------------------------------------------------------------------

__device__
inline Vec3_cu Precomputed_prim::gf(const Point_cu& x) const
{
    return Precomputed_env::fetch_gradient(_id, x);
}

// -----------------------------------------------------------------------------

__device__
inline float Precomputed_prim::fngf(Vec3_cu& gf, const Point_cu& x) const
{
    return Precomputed_env::fetch_potential_and_gradient(_id, x, gf);
}


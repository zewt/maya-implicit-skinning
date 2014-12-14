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
#ifndef HRBF_ENV_INTERFACE_HPP__
#define HRBF_ENV_INTERFACE_HPP__

/**
    Enable other compilation unit to bind and unbind hrbf texture when included
    this file.

    Because texture and kernels using them are to be in the same file scope
    (i.e same compilation unit) this means texture header rbf_hermite_tex.hpp
    cannot be included at several locations because it would create multiple
    declaration of global texture reference.
*/
// =============================================================================
namespace HRBF_env{
// =============================================================================

//extern void init_tex();
//extern void bind();
//extern void unbind();

}// END HRBF_ENV NAMESPACE =====================================================

#endif // HRBF_ENV_INTERFACE_HPP__

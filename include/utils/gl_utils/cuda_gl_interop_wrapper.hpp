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
#ifndef CUDA_GL_INTEROP_WRAPPER_HPP__
#define CUDA_GL_INTEROP_WRAPPER_HPP__

/**
 * @file cuda_gl_interop_wrapper.hpp
 * @deprecated
 *
 *  This include file exports the functions in cuda_gl_interop.h so they can be
    used in non-cuda .cpp files
 */

void cuda_register_buffer_object (GLuint bufObj);

void cuda_unregister_buffer_object (GLuint bufobj);

void cuda_map_buffer_object(void** devPtr, GLuint bufObj);

void cuda_unmap_buffer_object(GLuint bufObj);

#endif // CUDA_GL_INTEROP_WRAPPER_HPP__

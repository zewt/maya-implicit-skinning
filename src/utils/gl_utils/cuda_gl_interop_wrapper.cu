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
#include "port_cuda_gl_interop.h"
#include "cuda_utils.hpp"

#ifndef NDEBUG
#include <map>
std::map<GLuint, GLuint> registered;
#endif


void cuda_register_buffer_object (GLuint bufObj)
{
#ifndef NDEBUG
    static int acc = 0;
    // Check it has not already been registered
    std::map<GLuint, GLuint>::iterator it;
    it = registered.find(bufObj);
    assert(it == registered.end());
    registered[bufObj] = acc;
    acc++;
#endif

    CUDA_SAFE_CALL(cudaGLRegisterBufferObject(bufObj));
}

void cuda_unregister_buffer_object (GLuint bufObj)
{
#ifndef NDEBUG
    // Check it has been registered
    std::map<GLuint, GLuint>::iterator it;
    it = registered.find(bufObj);
    assert(it != registered.end());
    registered.erase(it);
#endif
    CUDA_SAFE_CALL(cudaGLUnregisterBufferObject(bufObj));
}

void cuda_map_buffer_object(void** devPtr, GLuint bufObj){
    CUDA_SAFE_CALL(cudaGLMapBufferObject(devPtr, bufObj));
}

void cuda_unmap_buffer_object(GLuint bufObj){
    CUDA_SAFE_CALL(cudaGLUnmapBufferObject(bufObj));
}

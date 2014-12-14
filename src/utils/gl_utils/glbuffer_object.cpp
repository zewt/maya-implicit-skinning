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
#include "glbuffer_object.hpp"

// -----------------------------------------------------------------------------

GlBuffer_obj::GlBuffer_obj(int type) :
    _is_cuda_registered(false),
    _size_buffer(0),
    _type(type),
    _cuda_rsrc(0)
{
    _data_ratio = guess_data_ratio(_type);
    glAssert( glGenBuffers(1, &_buffer_id) );
}

// -----------------------------------------------------------------------------

GlBuffer_obj::GlBuffer_obj(int nb_elt, int type, GLenum mode):
    _is_cuda_registered(false),
    _size_buffer(nb_elt),
    _type(type),
    _cuda_rsrc(0)
{
    _data_ratio = guess_data_ratio(type);
    glAssert( glGenBuffers(1, &_buffer_id) );
    glAssert( glBindBuffer(_type, _buffer_id) );
    glAssert( glBufferData(_type, nb_elt * _data_ratio, NULL, mode) );
    glAssert( glBindBuffer(_type, 0) );
}

// -----------------------------------------------------------------------------

GlBuffer_obj::~GlBuffer_obj() {
    assert(!_is_cuda_registered);
    glAssert( glDeleteBuffers(1, &_buffer_id) );
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::bind() const {
    glAssert( glBindBuffer(_type, _buffer_id) );
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::unbind() const {
    glAssert( glBindBuffer(_type, 0) );
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::set_data(int nb_elt,
                            const GLvoid* data,
                            GLenum mode)
{
    bind();
    _size_buffer = nb_elt;
    glAssert( glBufferData(_type, nb_elt * _data_ratio, data, mode) );
    unbind();
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::get_data(int offset,
                            int nb_elt,
                            GLvoid* data) const
{
    bind();
    glAssert( glGetBufferSubData(_type, offset, nb_elt * _data_ratio, data) );
    unbind();
}

// -----------------------------------------------------------------------------

bool GlBuffer_obj::unmap() const {
    bind();
    bool state = glUnmapBuffer(_type) ? true : false;
    GL_CHECK_ERRORS();
    return state;
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::cuda_register(unsigned flag_read_write){
    assert(!_is_cuda_registered);
    assert(_size_buffer > 0);

    _is_cuda_registered = true;
    CUDA_SAFE_CALL( cudaGraphicsGLRegisterBuffer(&_cuda_rsrc, _buffer_id, flag_read_write) );
    //cuda_register_buffer_object(_buffer_id);
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::cuda_unregister(){
    assert(_is_cuda_registered);
    assert(_size_buffer > 0);
    _is_cuda_registered = false;
//    cuda_unregister_buffer_object(_buffer_id);
    CUDA_SAFE_CALL( cudaGraphicsUnregisterResource( _cuda_rsrc ) );
    //_cuda_rsrc = 0;
}

// -----------------------------------------------------------------------------

void GlBuffer_obj::cuda_unmap() {
    assert(_is_cuda_registered);
    assert(_size_buffer > 0);
//    cuda_unmap_buffer_object(_buffer_id);
    CUDA_SAFE_CALL( cudaGraphicsUnmapResources(1, &_cuda_rsrc) );
}

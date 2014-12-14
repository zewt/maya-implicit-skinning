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
#ifndef BUFFER_OBJECT_HPP__
#define BUFFER_OBJECT_HPP__

#include "port_glew.h"
#include "cuda_gl_interop_wrapper.hpp"
#include "glassert.h"
#include <cassert>


//struct cudaGraphicsResource;

#include <cuda_runtime_api.h>
#include <cuda_gl_interop.h>
#include "cuda_utils_common.hpp"


/** Class of OpenGL buffer objects.
    The type is either GL_ARRAY_BUFFER or GL_ELEMENT_ARRAY_BUFFER or
    GL_PIXEL_[UN]PACK_BUFFER (for PBOs)
*/

struct GlBuffer_obj {

    /// OpenGL context has to be created before this constructor is called,
    /// hence the "explicit"
    /// @param type : GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER,
    /// GL_PIXEL_PACK_BUFFER, GL_PIXEL_UNPACK_BUFFER
    explicit GlBuffer_obj(int type);

    /// @param mode  GL_STREAM_DRAW, GL_STREAM_READ, GL_STREAM_COPY,
    /// GL_STATIC_DRAW, GL_STATIC_READ, GL_STATIC_COPY,
    /// GL_DYNAMIC_DRAW, GL_DYNAMIC_READ, or GL_DYNAMIC_COPY
    explicit GlBuffer_obj(int nb_elt, int type, GLenum mode = GL_STREAM_DRAW);

    ~GlBuffer_obj();

    /// Bind the buffer object
    void bind() const;

    /// Unbind the buffer object
    void unbind() const;

    /// Allocate and upload data for the buffer object
    /// @param data the pointer to the data to be copied or null if no data is
    /// to be copied (allocation only)
    /// @param mode  GL_STREAM_DRAW, GL_STREAM_READ, GL_STREAM_COPY,
    /// GL_STATIC_DRAW, GL_STATIC_READ, GL_STATIC_COPY,
    /// GL_DYNAMIC_DRAW, GL_DYNAMIC_READ, or GL_DYNAMIC_COPY
    void set_data(int nb_elt,
                  const GLvoid* data,
                  GLenum mode = GL_STREAM_DRAW);

    /// Download data from the buffer object
    void get_data(int offset,
                  int nb_elt,
                  GLvoid* data) const;


    /// Map the buffer object to some pointer on host memory
    ///  @param var the pointer in host memory mapping to the vbo
    ///  @param access_mode can be : GL_READ_ONLY, GL_WRITE_ONLY or GL_READ_WRITE.
    template <class T>
    inline void map_to(T*& var, GLenum access_mode) const {
        bind();
        assert(_size_buffer > 0);
        var = (T*)glMapBuffer(_type, access_mode);
        GL_CHECK_ERRORS();
    }

    /// Unmap the buffer object
    bool unmap() const;

    // =========================================================================
    /// @name Cuda interop
    // =========================================================================

    /// Register the buffer object for it to be usable by cuda
    /// @param flag_read_write : cudaGraphicsRegisterFlagsNone (read and write),
    /// cudaGraphicsRegisterFlagsReadOnly (read only),
    /// cudaGraphicsRegisterFlagsWriteDiscard (write only knowing everything will
    /// be written over)
    void cuda_register(unsigned flag_read_write = cudaGraphicsRegisterFlagsNone);

    /// Unregister the buffer object
    void cuda_unregister();

    /// Map the buffer object to some cuda device pointer
    /// @warning the buffer object has to be registered first with cuda_register()
    template <class T>
    inline void cuda_map_to(T*& var) {
        assert(_is_cuda_registered);
        assert(_size_buffer > 0);
        //cuda_map_buffer_object((void**) &var, _buffer_id);
        size_t s = _data_ratio * _size_buffer;
        CUDA_SAFE_CALL( cudaGraphicsMapResources(1, &_cuda_rsrc) );
        CUDA_SAFE_CALL( cudaGraphicsResourceGetMappedPointer((void**) &var, &s, _cuda_rsrc) );
    }

    /// Unmap the buffer object
    void cuda_unmap();

    // =========================================================================
    /// @name Getter & Setters
    // =========================================================================

    /// @return buffer object ID OpenGL
    inline int get_id() const { return _buffer_id; }

    /// @return number of elements inside the BO
    inline int size() const { return _size_buffer; }

private:
    int guess_data_ratio(int type)
    {
        return
        (type == GL_ARRAY_BUFFER)? sizeof(GLfloat) :
        (type == GL_ELEMENT_ARRAY_BUFFER) ||
        (type == GL_PIXEL_PACK_BUFFER) ||
        (type == GL_PIXEL_UNPACK_BUFFER) ? sizeof(GLuint): 1;
    }

    // =========================================================================
    /// @name Attributes
    // =========================================================================

    GLuint _buffer_id;              ///< OpenGl id of the buffer object
    bool   _is_cuda_registered;     ///< does the buffer cuda registered
    int    _size_buffer;            ///< number of elements in the buffer
    int    _data_ratio;             ///< Byte size of a buffer element

    /// buffer type: GL_ARRAY_BUFFER (usually for vertex attribs such as
    /// position, normals etc.); GL_ELEMENT_ARRAY_BUFFER (usually describe
    /// faces). GL_PIXEL_UNPACK_BUFFER and GL_PIXEL_PACK_BUFFER for pixel
    /// buffer objects (PBO)
    int    _type;

    cudaGraphicsResource_t _cuda_rsrc;
};

#endif // BUFFER_OBJECT_HPP__

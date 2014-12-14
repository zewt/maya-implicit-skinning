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
#ifndef GL_TEX_2D_HPP__
#define GL_TEX_2D_HPP__

#include "glsave.hpp"

/** @class GlTex2D
    @brief Utility to handle opengl 2D textures in a OO manner

    Simple opengl 2D texture handling.
*/
class GlTex2D {
public:

    /// Initialize texture with default parameters no memory allocation on GPU
    GlTex2D();

    /// Constructor
    /// @param filter The following symbolic values are accepted: GL_NEAREST,
    /// GL_LINEAR, GL_NEAREST_MIPMAP_NEAREST,  GL_LINEAR_MIPMAP_NEAREST,
    /// GL_NEAREST_MIPMAP_LINEAR, GL_LINEAR_MIPMAP_LINEAR

    /// @param wrap_mode The following symbolic values are accepted: GL_CLAMP,
    /// GL_CLAMP_TO_BORDER, GL_CLAMP_TO_EDGE, GL_MIRRORED_REPEAT, or GL_REPEAT.

    /// @param internal_format GL_COLOR_INDEX, GL_RED, GL_GREEN, GL_BLUE,
    /// GL_ALPHA, GL_RGB, GL_BGR, GL_RGBA, GL_BGRA, GL_LUMINANCE,
    /// GL_DEPTH_COMPONENT and GL_LUMINANCE_ALPHA.
    GlTex2D(int width, int height, int border,
            GLuint filter, GLuint wrap_mode, GLenum internal_format);

    /// dtor
    ~GlTex2D();

    /// Change size of the texture. The change will take effect untill the next
    /// call of 'allocate()'.
    void set_size(int width, int height);

    /// Allocate memory on GPU.
    /// @param data : pointer on a 1D array in cpu used to fill the  texture.
    /// Type an format of data are specified  with buffer_type buffer_format
    /// If no pointer is given the memory is allocated but not initialize. It
    /// is also possible to bind a pixel buffer object before calling the method
    /// texture will be fill with the PBO in a asynchronous way.
    /// @param buffer_type the type of the 'data' pointer. The following
    /// symbolic values are accepted: GL_UNSIGNED_BYTE, GL_BYTE, GL_BITMAP,
    /// GL_UNSIGNED_SHORT, GL_SHORT, GL_UNSIGNED_INT, GL_INT, GL_FLOAT,
    /// GL_UNSIGNED_BYTE_3_3_2, GL_UNSIGNED_BYTE_2_3_3_REV,
    /// GL_UNSIGNED_SHORT_5_6_5, GL_UNSIGNED_SHORT_5_6_5_REV,
    /// GL_UNSIGNED_SHORT_4_4_4_4, GL_UNSIGNED_SHORT_4_4_4_4_REV,
    /// GL_UNSIGNED_SHORT_5_5_5_1, GL_UNSIGNED_SHORT_1_5_5_5_REV,
    /// GL_UNSIGNED_INT_8_8_8_8, GL_UNSIGNED_INT_8_8_8_8_REV,
    /// GL_UNSIGNED_INT_10_10_10_2, and GL_UNSIGNED_INT_2_10_10_10_REV
    /// @param format GL_ALPHA, GL_ALPHA4, GL_ALPHA8, GL_ALPHA12, GL_ALPHA16,
    /// GL_COMPRESSED_ALPHA, GL_COMPRESSED_LUMINANCE, GL_COMPRESSED_LUMINANCE_ALPHA,
    /// GL_COMPRESSED_INTENSITY, GL_COMPRESSED_RGB, GL_COMPRESSED_RGBA,
    /// GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT24,
    /// GL_DEPTH_COMPONENT32, GL_LUMINANCE, GL_LUMINANCE4, GL_LUMINANCE8,
    /// GL_LUMINANCE12, GL_LUMINANCE16, GL_LUMINANCE_ALPHA, GL_LUMINANCE4_ALPHA4,
    /// GL_LUMINANCE6_ALPHA2, GL_LUMINANCE8_ALPHA8, GL_LUMINANCE12_ALPHA4,
    /// GL_LUMINANCE12_ALPHA12, GL_LUMINANCE16_ALPHA16, GL_INTENSITY,
    /// GL_INTENSITY4, GL_INTENSITY8, GL_INTENSITY12, GL_INTENSITY16, GL_R3_G3_B2,
    /// GL_RGB, GL_RGB4, GL_RGB5, GL_RGB8, GL_RGB10, GL_RGB12, GL_RGB16, GL_RGBA,
    /// GL_RGBA2, GL_RGBA4, GL_RGB5_A1, GL_RGBA8, GL_RGB10_A2, GL_RGBA12, GL_RGBA16,
    /// GL_SLUMINANCE, GL_SLUMINANCE8, GL_SLUMINANCE_ALPHA, GL_SLUMINANCE8_ALPHA8,
    /// GL_SRGB, GL_SRGB8, GL_SRGB_ALPHA, or GL_SRGB8_ALPHA8.
    /// @warning don't forget to bind()
    void allocate(GLenum buffer_type, GLenum buffer_format, const void* data = 0);

    void bind();

    static void unbind();

    /// Get the id of the 2D texture currently binded.
    static GLuint get_currently_bind();

    /// Copy the screen buffer into the texture.
    /// @warning don't forget to bind().
    /// Morevover it is an extremly slow operation
    void copy_screen_buffer();

    void configure_as_shadow_map();

    GLuint id();

private:
    // -------------------------------------------------------------------------
    /// @name Class Tools
    // -------------------------------------------------------------------------
    /// Setup texture wrapping mode and filter
    void setup();

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    GLuint  _id;
    GLuint  _wrap;
    GLuint  _filter;
    GLint   _border;
    GLfloat _color[4];        ///< border color
    int     _internal_format;
    int     _width;
    int     _height;

    bool _use_mipmap;
};


#endif

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
#include "gltex2D.hpp"
#include <cassert>


GlTex2D::GlTex2D():
    _wrap(GL_CLAMP),
    _filter(GL_LINEAR),
    _border(0),
    _internal_format(GL_FLOAT),
    _width(-1),
    _height(-1),
    _use_mipmap(false)
{
    _color[0] = _color[1] = _color[2] = _color[3] = 0.f;
    GLTextureBinding2DSave save_tex_binding;
    bind();
    glAssert( glGenTextures(1, &_id) );
    setup();
}

//------------------------------------------------------------------------------

GlTex2D::GlTex2D(int width, int height, int border,
                 GLuint filter, GLuint wrap_mode, GLenum internal_format) :
    _wrap(wrap_mode),
    _filter(filter),
    _border(border),
    _internal_format(internal_format),
    _width(width),
    _height(height)
{
    _color[0] = _color[1] = _color[2] = _color[3] = 0.f;
    GLTextureBinding2DSave save_tex_binding;
    glAssert( glGenTextures(1, &_id) );
    bind();

    _use_mipmap = (filter == GL_NEAREST_MIPMAP_NEAREST) ||
            (filter == GL_LINEAR_MIPMAP_NEAREST) ||
            (filter == GL_NEAREST_MIPMAP_LINEAR) ||
            (filter == GL_LINEAR_MIPMAP_LINEAR );

    setup();
}

//------------------------------------------------------------------------------

GlTex2D::~GlTex2D(){
    GLTextureBinding2DSave save_tex_binding;
    GlTex2D::unbind();
    glAssert( glDeleteTextures(1, &_id) );
}

//------------------------------------------------------------------------------

void GlTex2D::set_size(int width, int height){
    _width = width; _height = height;
}

//------------------------------------------------------------------------------

void GlTex2D::allocate(GLenum buffer_type, GLenum buffer_format, const void* data)
{
    assert(_id == GlTex2D::get_currently_bind());
    glAssert( glTexImage2D(GL_TEXTURE_2D,
                           0,                // Mimap level
                           _internal_format,
                           _width,
                           _height,
                           _border,          // border zise
                           buffer_format,
                           buffer_type,
                           data              // data (current tex binding if 0)
                           ));
}

//------------------------------------------------------------------------------

void GlTex2D::bind(){
    glAssert( glBindTexture(GL_TEXTURE_2D, _id) );
}

//------------------------------------------------------------------------------

void GlTex2D::unbind(){
    glAssert( glBindTexture(GL_TEXTURE_2D, 0) );
}

//------------------------------------------------------------------------------

GLuint GlTex2D::get_currently_bind(){
    int id;
    glAssert( glGetIntegerv(GL_TEXTURE_BINDING_2D, &id) );
    return id;
}

//------------------------------------------------------------------------------

void GlTex2D::copy_screen_buffer()
{
    assert(_id == GlTex2D::get_currently_bind());
    glCopyTexImage2D(GL_TEXTURE_2D,
                     0,             // Mimap level
                     _internal_format,
                     0,             // starting x coordinates
                     0,             // starting y coordinates
                     _width,        // ending x coordinates
                     _height,       // ending y coordinates
                     _border);      // Texture border size
}

//------------------------------------------------------------------------------

void GlTex2D::configure_as_shadow_map()
{
    assert(_id == GlTex2D::get_currently_bind());
    // result of the fetch set to the result of the comparison between
    // the texture at coordinates (s/q, t/q) and r/q
    // (r being the current rendered depth i.e the depth of the current fragment
    // transformed into light space )
    glAssert(glTexParameteri(GL_TEXTURE_2D,
                             GL_TEXTURE_COMPARE_MODE,
                             GL_COMPARE_R_TO_TEXTURE));
    glAssert(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL));
    // Where the result is written GL_INTENSITY (DDDD) GL_ALPHA (000D)
    // GL_LUMINANCE (DDD1)
    glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
}

//------------------------------------------------------------------------------

GLuint GlTex2D::id(){ return _id; }

//------------------------------------------------------------------------------

void GlTex2D::setup()
{
    assert(_id == GlTex2D::get_currently_bind());
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, _wrap) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, _wrap) );
    GLenum mag_filter;
    if(_filter == GL_LINEAR ||
       _filter == GL_LINEAR_MIPMAP_NEAREST ||
       _filter ==  GL_LINEAR_MIPMAP_LINEAR)
    {
        mag_filter = GL_LINEAR;
    }
    else
        mag_filter = GL_NEAREST;

    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, _filter)    );

    glAssert(glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, _color));


    if(_use_mipmap) glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
}

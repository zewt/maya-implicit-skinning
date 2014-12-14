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
#include "render_context.hpp"

#include "gltex2D.hpp"
#include "glpick_fbo.hpp"

// -----------------------------------------------------------------------------

Render_context::Render_context(int w, int h):
    _pbo_color(0),
    _pbo_depth(0),
    _d_img_buffer(0),
    _d_bloom_buffer(0),
    _d_rendu_buf(0),
    _d_rendu_depth_buf(0),
    _plain_phong(false),
    _textures(true),
    _draw_mesh(true),
    _raytrace(false),
    _skeleton(false),
    _rest_pose(false)
{
    _peeler    = new Peeler();
    _frame_tex = new GlTex2D(MULTISAMPX * w, MULTISAMPY * h,
                             0, GL_LINEAR, GL_CLAMP, GL_RGBA);
    _picker    = new GlPick_FBO(w, h);

    allocate(w, h);
}

// -----------------------------------------------------------------------------

Render_context::~Render_context()
{
    _pbo_color->cuda_unregister();
    _pbo_depth->cuda_unregister();
    delete _pbo_color;
    delete _pbo_depth;
    delete _frame_tex;
    delete _peeler;
    delete _picker;

    Cuda_utils::free_d( _d_img_buffer      );
    Cuda_utils::free_d( _d_bloom_buffer    );
    Cuda_utils::free_d( _d_rendu_buf       );
    Cuda_utils::free_d( _d_rendu_depth_buf );
}

// -----------------------------------------------------------------------------

void Render_context::reshape(int w, int h)
{
    allocate(w, h);
}

// -----------------------------------------------------------------------------

void Render_context::allocate(int width, int height)
{
    _width  = width;
    _height = height;

    _frame_tex->bind();
    _frame_tex->set_size(MULTISAMPX * width, MULTISAMPY * height);
    _frame_tex->allocate(GL_UNSIGNED_BYTE, GL_RGBA);
    GlTex2D::unbind();

    if(_pbo_color != 0) _pbo_color->cuda_unregister();
    if(_pbo_depth != 0) _pbo_depth->cuda_unregister();

    /*
    delete _pbo_color;
    delete _pbo_depth;

    _pbo_color = new BufferObject<GL_PIXEL_UNPACK_BUFFER>(MULTISAMPX*width*MULTISAMPY*height);
    _pbo_depth = new BufferObject<GL_PIXEL_UNPACK_BUFFER>(width*height);
    */
    if( _pbo_color != 0 ) _pbo_color->set_data(MULTISAMPX*width*MULTISAMPY*height, 0);
    else                  _pbo_color = new GlBuffer_obj(MULTISAMPX*width*MULTISAMPY*height, GL_PIXEL_UNPACK_BUFFER);

    if(_pbo_depth != 0) _pbo_depth->set_data(width*height, 0);
    else                _pbo_depth = new GlBuffer_obj(width*height, GL_PIXEL_UNPACK_BUFFER);

    // Register pbos
    _pbo_color->cuda_register();
    _pbo_depth->cuda_register();

    Cuda_utils::free_d( _d_img_buffer      );
    Cuda_utils::free_d( _d_bloom_buffer    );
    Cuda_utils::free_d( _d_rendu_buf       );
    Cuda_utils::free_d( _d_rendu_depth_buf );

    Cuda_utils::malloc_d(_d_img_buffer     , width * MULTISAMPX * height * MULTISAMPY * 2 );
    Cuda_utils::malloc_d(_d_bloom_buffer   , width * MULTISAMPX * height * MULTISAMPY * 2 );
    Cuda_utils::malloc_d(_d_rendu_buf      , width * MULTISAMPX * height * MULTISAMPY     );
    Cuda_utils::malloc_d(_d_rendu_depth_buf, width * height);

    _peeler->reinit_depth_peeling(width, height);

    _picker->resize(width, height);
}

// -----------------------------------------------------------------------------

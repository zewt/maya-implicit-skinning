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
#include "glpick_fbo.hpp"

#include "glfbo.hpp"
#include "gltex2D.hpp"
#include "shader.hpp"

#include <cassert>
#include <limits>
#include <vector>

// =============================================================================
namespace Shader_picking {
// =============================================================================

Shader_prog* g_shader = 0;

bool is_shaders_init = false;

void delete_shaders()
{
    delete g_shader;
    g_shader = 0;

    is_shaders_init = false;
}

// -----------------------------------------------------------------------------

void init_shaders()
{
    delete_shaders();

    Shader vs("src/shaders/picking.vert", GL_VERTEX_SHADER  );
    Shader fs("src/shaders/picking.frag", GL_FRAGMENT_SHADER);

    g_shader = new Shader_prog(vs, fs);
    g_shader->link();

    is_shaders_init = true;
}

}// END SHADER PICKING =========================================================

// -----------------------------------------------------------------------------

GlPick_FBO::GlPick_FBO(int width, int height) :
    _is_pick_init(false)
{
    _fbo = new GlFbo(false, GlFbo::COLOR|GlFbo::DEPTH);

    _color_tex = new GlTex2D(width, height, 0, GL_NEAREST, GL_CLAMP, GL_R32UI );
    _color_tex->bind();
    _color_tex->allocate( GL_UNSIGNED_INT, GL_RED_INTEGER );

    _depth_tex = new GlTex2D(width, height, 0, GL_NEAREST, GL_CLAMP, GL_DEPTH_COMPONENT);
    _depth_tex->bind();
    _depth_tex->allocate( GL_FLOAT, GL_DEPTH_COMPONENT );
    GlTex2D::unbind();

    _fbo->bind();
    _fbo->set_render_size(width, height);
    _fbo->record_tex_attachment( GL_COLOR_ATTACHMENT0, _color_tex->id() );
    _fbo->record_tex_attachment( GL_DEPTH_ATTACHMENT , _depth_tex->id() );
    _fbo->update_attachments();
    assert( _fbo->check() );
    _fbo->unbind();

    if( !Shader_picking::is_shaders_init )
        Shader_picking::init_shaders();
}

// -----------------------------------------------------------------------------

GlPick_FBO::~GlPick_FBO()
{
    delete _fbo;
    delete _color_tex;
    delete _depth_tex;
    _fbo       = 0;
    _color_tex = 0;
    _depth_tex = 0;
}

// -----------------------------------------------------------------------------

void GlPick_FBO::begin(int x, int y, GLfloat* mvp_mat)
{
    _x = x;
    _y = _fbo->height() - y;

    glAssert( glClearColor( 0, 0, 0, 0 ) );
    _fbo->use_as_target();
    _fbo->clear_buffers(GlFbo::COLOR|GlFbo::DEPTH);

    Shader_picking::g_shader->use();
    Shader_picking::g_shader->set_mat4x4("MVP", mvp_mat);

    _is_pick_init = true;
}

// -----------------------------------------------------------------------------

int GlPick_FBO::end()
{
    assert( _is_pick_init );
    _is_pick_init = false;
    Shader_picking::g_shader->unuse();

    std::vector<unsigned> id(20, 0);

    _fbo->unbind();
    glAssert( glBindFramebufferEXT(GL_READ_FRAMEBUFFER, _fbo->id()) );
    glAssert( glReadBuffer(GL_COLOR_ATTACHMENT0) );
    glAssert( glReadPixels(_x, _y, 1, 1, GL_RED_INTEGER, GL_UNSIGNED_INT, &(id[0])) );

    glAssert( glReadBuffer(GL_NONE) );
    glAssert( glBindFramebufferEXT(GL_READ_FRAMEBUFFER, 0) );

    return id[0]-1;
}

// -----------------------------------------------------------------------------

void GlPick_FBO::set_name(unsigned id)
{
    assert( _is_pick_init );
    Shader_picking::g_shader->set_uniform("g_object_idx", (int)id+1);
}

// -----------------------------------------------------------------------------

void GlPick_FBO::resize(int w, int h)
{
    _color_tex->bind();
    _color_tex->set_size(w, h);
    _color_tex->allocate( GL_UNSIGNED_INT, GL_RED_INTEGER );

    _depth_tex->bind();
    _depth_tex->set_size(w, h);
    _depth_tex->allocate( GL_FLOAT, GL_DEPTH_COMPONENT );
    GlTex2D::unbind();

    _fbo->bind();
    _fbo->set_render_size(w, h);
    _fbo->update_attachments( true );
    _fbo->unbind();
}

// -----------------------------------------------------------------------------

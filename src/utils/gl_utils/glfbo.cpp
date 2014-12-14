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
#include "glfbo.hpp"

#include <iostream>

GLuint GlFbo::_currently_bound = 0;
GLuint GlFbo::_current_target = 0;
GLDrawBufferSave GlFbo::_draw_buffer_save (false);
std::map<GLuint, GlFbo *> GlFbo::_id_to_FBO;
std::stack<GlFbo *> GlFbo::_use_as_target_stack;

//------------------------------------------------------------------------------

GlFbo::GlFbo(bool main, Components_flags comps) :
        _id(0),
        _width(-1),
        _height(-1),
        _rbo(0),   //the 0 value is reserved for openGL => invalid.
        _is_depth_attachment(false),
        _is_stencil_attachment(false),
        _components(comps),
        _stencil_mask(0xFF),
        _modify_color(true),
        _modify_Z(true),
        _modify_stencil(true)

{
    assert( (!main) || (_id_to_FBO.find( 0 ) == _id_to_FBO.end() ) );

    if (!main) glAssert( glGenFramebuffersEXT(1, &_id) );

    if (_id != 0)
    {
        if ( (_components & STENCIL) == STENCIL)
            // pack all in one rbo (creating two separate rbos seems not to be
            // supported, even if in the spec...) obliged to have a depth
            // component, even if not attached (GL_STENCIL_INDEX isn't supported)
            _rbo_format = GL_DEPTH24_STENCIL8_EXT;
        else
            if ( (_components & DEPTH) == DEPTH)
                _rbo_format = GL_DEPTH_COMPONENT24;
    }

    _id_to_FBO[_id] = this;
}

//------------------------------------------------------------------------------

GlFbo::~GlFbo()
{
    Scoped_binder binder(_id);

    if(_rbo != 0) glAssert( glDeleteRenderbuffersEXT(1, &_rbo) );
    if(_id  != 0) glAssert( glDeleteFramebuffersEXT (1, &_id ) );

    _id_to_FBO.erase( _id_to_FBO.find (_id) );
}


//------------------------------------------------------------------------------

void GlFbo::update_RBO()
{
    if (_id  == 0) return;

    if (_rbo == 0) glAssert (glGenRenderbuffersEXT (1, &_rbo));

    glAssert (glBindRenderbufferEXT (GL_RENDERBUFFER_EXT, _rbo));
    glAssert (glRenderbufferStorageEXT (GL_RENDERBUFFER_EXT, _rbo_format, _width, _height));
    glAssert (glBindRenderbufferEXT (GL_RENDERBUFFER_EXT, 0));

    if( ( (_components & DEPTH) == DEPTH) && !_is_depth_attachment)
    {
        glAssert(glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                              GL_DEPTH_ATTACHMENT_EXT,
                                              GL_RENDERBUFFER_EXT,
                                              _rbo));
    }

    if( ( (_components & STENCIL) == STENCIL) && !_is_stencil_attachment)
    {
        glAssert(glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                              GL_STENCIL_ATTACHMENT_EXT,
                                              GL_RENDERBUFFER_EXT,
                                              _rbo));
    }
}

//------------------------------------------------------------------------------

void GlFbo::set_render_size(int width, int height)
{
    _width  = width;
    _height = height;

    if (_id != 0)
    {
        assert_bound();
        if ( (_components & (DEPTH | STENCIL)) != 0)
            update_RBO();
    }
}

//------------------------------------------------------------------------------

void GlFbo::bind ()
{
    if(_currently_bound != _id)
    {
        glAssert( glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _id) );
        _currently_bound = _id;
    }
}

//------------------------------------------------------------------------------

void GlFbo::unbind ()
{
    //      std::cerr<<"FBO unbound: "<<m_id<<" => FBO bound : 0"<<std::endl;
    if ( (_components & COLOR) == 0) {
        _draw_buffer_save.restore();
    }

    if (_currently_bound != 0)
    {
        glAssert (glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0));
        _currently_bound = 0;
    }
}

//------------------------------------------------------------------------------
bool GlFbo::check () const
{
    //ensure the status is ok

    //ScopedBinder binder (m_id);
    assert_bound();

    GLenum err = glCheckFramebufferStatusEXT (GL_FRAMEBUFFER_EXT);
    if (err != GL_FRAMEBUFFER_COMPLETE_EXT) {
        std::cerr << "FBO::check() : FBO " << _id << " not complete (error = " << err << ") : ";

        switch (err) {
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
            std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
            std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
            std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
            std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
            std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
            std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT";
            break;
        case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
            std::cerr << "GL_FRAMEBUFFER_UNSUPPORTED_EXT";
            break;
        default:
            std::cerr << "Unknown ERROR";
        }
        std::cerr << std::endl;
    }

    return (err == GL_FRAMEBUFFER_COMPLETE_EXT);
}

//------------------------------------------------------------------------------

void GlFbo::use_as_target()
{
    bind();

    // when no color texture is attached, glDrawBuffer must be set to GL_NONE
    // in order to get a valid FBO: calling check() before doing
    // glDrawBuffer(GL_NONE); leads to an incomplete draw buffer.
    // Sadly, it seems that glDrawBuffer() affects not only the currently
    // bound FBO, but also the main FBO, so its value must be set to NONE only
    // when the shadow map is computed, and reset after, when the unbinding
    // is done.
    if( (_components & COLOR) == 0)
    {
        _draw_buffer_save.save();
        glAssert( glDrawBuffer(GL_NONE) );
    }

    assert( check() );

    //the masks are for all FBOs (linked to the global GL state, not bound to a
    // specific FBO) => change them each time a FBO is bound to be sure its
    // parameters are correctly set.
    apply_modify( ALL );

    //ensure all the FBO will be filled
    glAssert (glViewport (0, 0, _width, _height));
}

//------------------------------------------------------------------------------

void GlFbo::apply_modify(Components_flags comps)
{
    if (comps & COLOR) {
        if (_modify_color) glColorMask (GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        else               glColorMask (GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    }

    if (comps & DEPTH) {
        if (_modify_Z)glDepthMask (GL_TRUE);
        else          glDepthMask (GL_FALSE);
    }

    if (comps & STENCIL) {
        if (_modify_stencil) glStencilMask (_stencil_mask);
        else                 glStencilMask (0);
    }
}

//------------------------------------------------------------------------------

void GlFbo::set_blending (GLenum funcSrc, GLenum funcDst, GLenum eq)
{
    //ScopedBinder binder (m_id);
    assert_bound ();

    glAssert( glBlendFunc(funcSrc, funcDst) );
    glAssert( glBlendEquation(eq)           );
}

//------------------------------------------------------------------------------

void GlFbo::record_tex_attachment(GLenum attach_point, GLuint tex_id)
{
    if (_id != 0) _to_attach[attach_point] = tex_id;
}

//------------------------------------------------------------------------------

void GlFbo::record_tex_detachment(GLenum attach_point)
{
    if (_id != 0) _to_detach.push_back(attach_point);
}

//------------------------------------------------------------------------------

void GlFbo::update_attachments(bool re_attach_all)
{
    if (_id == 0) return;

    assert_bound();

    bool old_is_depth_attachment   = _is_depth_attachment;
    bool old_is_stencil_attachment = _is_stencil_attachment;

    // detach first, to avoid detaching a newly attached texture after,
    // when replacing a texture by another one
    std::list<GLenum>::const_iterator it;
    for( it = _to_detach.begin(); it != _to_detach.end(); ++it)
    {
        glAssert( glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                                            *it,
                                            GL_TEXTURE_2D,
                                            0,
                                            0));

        std::map<GLenum, GLuint>::iterator it_attach = _attachments.find(*it);

        if( it_attach != _attachments.end() )
            _attachments.erase( it_attach );

        if (*it == GL_DEPTH_ATTACHMENT_EXT) {
            _is_depth_attachment = false;
        } else if (*it == GL_STENCIL_ATTACHMENT_EXT) {
            _is_stencil_attachment = false;
        }
    }

    _to_detach.clear();

    if (re_attach_all)
    {
        // reattach existing textures, before adding the new ones to avoid
        // attaching them twice
        std::map<GLenum, GLuint>::const_iterator it;
        for(it = _attachments.begin(); it != _attachments.end(); ++it)
            glAssert(glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                                               it->first,
                                               GL_TEXTURE_2D,
                                               it->second,
                                               0));

    }

    std::map<GLenum, GLuint>::const_iterator it_attach;
    for(it_attach = _to_attach.begin(); it_attach != _to_attach.end(); ++it_attach)
    {
        // first unbind the RBO if it was attached before
        if(it_attach->first == GL_DEPTH_ATTACHMENT_EXT)
        {
            if( !old_is_depth_attachment )
                glAssert(glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                                      GL_DEPTH_ATTACHMENT_EXT,
                                                      GL_RENDERBUFFER_EXT,
                                                      0));

            _is_depth_attachment = true;
        }
        else if( it_attach->second == GL_STENCIL_ATTACHMENT_EXT )
        {
            if( !old_is_stencil_attachment )
                glAssert(glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                                      GL_STENCIL_ATTACHMENT_EXT,
                                                      GL_RENDERBUFFER_EXT, 0));

            _is_stencil_attachment = true;
        }

        glAssert(glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                                           it_attach->first,
                                           GL_TEXTURE_2D,
                                           it_attach->second,
                                           0));

        _attachments[ it_attach->first ] = it_attach->second;
    }

    _to_attach.clear();

    // bind the RBOs if a texture was bound to an attachment point and
    // no more texture is attached now
    if( old_is_depth_attachment && (!_is_depth_attachment) )
    {
        glAssert(glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                              GL_DEPTH_ATTACHMENT_EXT,
                                              GL_RENDERBUFFER_EXT,
                                              _rbo));
    }

    if( old_is_stencil_attachment && (!_is_stencil_attachment) )
    {
        glAssert(glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                              GL_STENCIL_ATTACHMENT_EXT,
                                              GL_RENDERBUFFER_EXT,
                                              _rbo));
    }

}

//------------------------------------------------------------------------------

void GlFbo::clear_buffers(Components_flags comps)
{
    assert_bound();

    Components_flags filtered_components = Components_flags (_components & comps);

    GLbitfield mask = 0;
    if ( (filtered_components & COLOR) == COLOR && _modify_color)
        mask |= GL_COLOR_BUFFER_BIT;

    if ( (filtered_components & DEPTH) == DEPTH && _modify_Z)
        mask |= GL_DEPTH_BUFFER_BIT;

    if ( (filtered_components & STENCIL) == STENCIL && _modify_stencil)
        mask |= GL_STENCIL_BUFFER_BIT;

    glClear( mask );
}

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
#ifndef GLFBO_HPP__
#define GLFBO_HPP__

#include "glsave.hpp"
#include <cassert>

#include <map>
#include <stack>
#include <list>

/**
 * @name GlFbo
   @brief basic abstraction for an OpenGL FBO.
   all the callers must ensure that the currently bound FBO is the one
   that is handled by the FBO object.
*/
class GlFbo {
public:

    typedef int Components_flags;
    /// @brief to describe the components used in the
    /// bit field 'Components_flags'
    enum Components {
        NONE    = 0,
        COLOR   = 1 << 0,
        DEPTH   = 1 << 1,
        STENCIL = 1 << 2,
        ALL     = COLOR | DEPTH | STENCIL
     };

private:

    GLuint _id;             ///< the fbo id
    int    _width, _height; ///< resolution

    /// the render buffer objects, necessary for depth and stencil when used but
    /// not attached to a texture
    GLuint _rbo;

    std::map<GLenum, GLuint> _attachments; ///< the map of attachments

    /// the map of attachment to perform (attachment points -> texture id)
    /// when update_attachments () is performed.
    std::map<GLenum, GLuint> _to_attach;

    /// the list of attachment points to detach
    std::list<GLenum> _to_detach;

    /// whether a texture is attached to the depth attachment point
    bool _is_depth_attachment;
    /// whether a texture is attached to the stencil attachment point
    bool _is_stencil_attachment;

    Components_flags _components;

    /// the mask for stencil buffer writing
    GLuint _stencil_mask;

    /// for individual temporary modification toggling
    bool _modify_color;
    bool _modify_Z;
    bool _modify_stencil;

    /// the RBO format to use,
    /// depending on whether depth or depth+stencil is used
    GLenum _rbo_format;

    /// id of the FBO currently bound
    static GLuint _currently_bound;
    /// id of the FBO used as target for rendering
    static GLuint _current_target;

    static GLDrawBufferSave _draw_buffer_save;

    /// to update the rbo and attach it.
    /// the FBO must have been bound before
    void update_RBO();

    /// @name ScopedBinder
    /// @brief to ease temporary binding.
    /// @warning : this is costly, limit use at max
    //------------------------
    class Scoped_binder {
    public:
        GLuint _old_bound;
        GLuint _bound;

        Scoped_binder (GLuint id) :
            _old_bound(GlFbo::_currently_bound),
            _bound (id)
        {
            if(id != _old_bound)
            {
                //std::cerr<<"FBO bound: "<<id_<<std::endl;
                glAssert (glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, id));
                _currently_bound = id;
            }
        }

        ~Scoped_binder ()
        {
            if(_old_bound != _bound)
            {
                //std::cerr<<"FBO unbound: "<<mBound<<std::endl;
                glAssert (glBindFramebufferEXT (GL_FRAMEBUFFER_EXT, _old_bound));
                _currently_bound = _old_bound;
            }
        }
    };

    friend class Scoped_binder;
    //------------------------

    /// a map of (Key: openGL FBO id) (Value: class GlFbo pointer)
    static std::map<GLuint, GlFbo*> _id_to_FBO;
    /// to manage stacks of useAsTarget
    static std::stack<GlFbo*> _use_as_target_stack;


public:
    /// constructor
    GlFbo(bool main, Components_flags comps);

    /// release the FBO
    ~GlFbo();

    GLuint id    () const { return _id;     }
    int    width () const { return _width;  }
    int    height() const { return _height; }

    /// to change the render size
    void set_render_size(int width, int height);

    /// bind FBO
    void bind();
    /// unbind FBO
    void unbind();

    /// to activate or disable writing a specific component.
    /// if the component is not part of _components, this does
    /// not do anything.
    void modify_color(bool write) {
        if( (_components & COLOR) == COLOR)
            _modify_color = write;
    }

    void modify_Z(bool write) {
        if ( (_components & DEPTH) == DEPTH)
            _modify_Z = write;
    }

    void modify_stencil(bool write) {
        if ( (_components & STENCIL) == STENCIL)
            _modify_stencil = write;
    }

    /// to change the stencil writing mask,
    /// used only if the components set by setUsedComponents() include it.
    void set_stencil_mask(GLuint stencil_mask) { _stencil_mask = stencil_mask;  }

    /// to apply the values of modify_X(), when one of this function
    /// has been called after use_as_target().
    /// @param comps to force the application to only a subset of the components
    void apply_modify(Components_flags comps);


    /// to use the FBO as target for rendering
    void use_as_target();

    /// to use the FBO as target, saving the previously used FBO
    void push_old_target_and_use() {
        _use_as_target_stack.push( _id_to_FBO[_currently_bound] );
        use_as_target();
    }

    /// to use the FBO pushed via the corresponding call to
    /// push_old_target_and_use() as target.
    static void pop_old_target_and_use() {
        assert( !_use_as_target_stack.empty() );
        _use_as_target_stack.top()->use_as_target();
        _use_as_target_stack.pop();
    }


    ///	to set the blending functions and equation of the FBO.
    /// this just set the equation and functions for the FBO, it
    /// does not modify the openGL state.
    void set_blending(GLenum funcSrc, GLenum funcDst, GLenum eq);

    /// to clear some of the buffers of the FBO
    /// @param comps :  buffer to clear
    void clear_buffers(Components_flags comps);

    // -------------------------------------------------------------------------
    /// @name Texture attachments
    // -------------------------------------------------------------------------

    /// to attach a texture to an attachment point.
    /// the attachment will be effective when update_attachments() is called.
    /// if 0, detach.  if called on the system FBO, does nothing.
    /// once updateAttachments() is called, any texture attachment is forgotten
    void record_tex_attachment(GLenum attach_point, GLuint tex_id);

    /// to detach a texture to an attachment point.
    /// the detachment will be effective when updateAttachments() is called.
    void record_tex_detachment (GLenum attach_point);

    /// apply attachments or detachments recorded using
    /// record_texture_(at|de)tachment(),
    /// @param re_attach_all : reattach even existing textures
    /// previously attached. This can be useful when they are reallocated or
    /// change resolution.
    void update_attachments(bool re_attach_all = false);

    // -------------------------------------------------------------------------
    /// @name Error checking
    // -------------------------------------------------------------------------

    /// once the FBO is completely setup, use this to verify it can be used.
    /// @return  true if the buffer is ok
    bool check() const;

    /// to ease verification
    void assert_bound() const { assert(_id == _currently_bound);  }

};

#endif // GLFBO_HPP__




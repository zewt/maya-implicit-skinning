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
#ifndef DEPTH_PEELING_HPP__
#define DEPTH_PEELING_HPP__

#include "glassert.h"
#include "glbuffer_object.hpp"
#include "shader.hpp"

/** These function performs depth peeling so that transparency is rendered in
    the right way
 */


/** The class that performs depth peeling with alpha blending
    @note : for now hardware antialiasing won't work with this because of
    the use of FBOs
*/
struct Peeler{

    class Render{
    public:
        virtual ~Render() {}
        /// Method to override in order to do the depth peeling of some objects
        virtual void draw_transc_objs() = 0;
    };

    Peeler();

    /// Sets the function used to draw the objects with depth peeling
    void set_render_func(Render* r);

    /// Draw a fullscreen quad
    static void draw_quad();

    /// Initialize color and depth buffers prior to depth peeling
    void set_background(int width, int height,
                        const GlBuffer_obj* pbo_color,
                        const GlBuffer_obj* pbo_depth);

    /// Do depth peelign with alpha blending
    void peel(float base_alpha);

    /// Initialize buffers
    void init_depth_peeling(int width, int height);

    /// Initialize buffers again (when window is resized)
    void reinit_depth_peeling(int width, int height);

    /// Erase textures FBO etc.
    void erase_gl_mem();

    // buffers used for depth peeling
    bool   _is_init;
    GLuint _depthTexId[2];
    GLuint _colorTexId[2];
    GLuint _fboId[2];
    GLuint _colorBlenderTexId;
    GLuint _colorBlenderFboId;
    GLuint _backgroundColorTexId;
    GLuint _backgroundDepthTexId;

    Render* _renderer;
};

#endif // DEPTH_PEELING_HPP__

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
#ifndef GLPICK_FBO_HPP__
#define GLPICK_FBO_HPP__

#include "port_glew.h"

class GlFbo;
class GlTex2D;

/**
 * @name GlPick_FBO
 * @brief Handling opengl picking with FBO
 *
 * GlPick_FBO helps selecting objects into a 3d scene displayed with openGl.
 * To find which object is under the mouse GlPick_FBO will draw each object
 * with a specific color into an offline render buffer (the FBO).
 *
 * Use case:
 * @code
 * GlPick_FBO pick(width, height);
 * pick.begin(int x, int y, GLfloat* mvp_mat);
 * // The the associated index to the next drawned object
 * pick.set_name( obj_index );
 *
 * // Draw object with openGL
 *
 * int selected_obj = pick.end(); // retreive selected index if any *
 * @endcode
 *
 * @note don't forget to resize the FBO with resize() when the viewport
 * resolution changes
 *
 * @warning you must disable the shaders when drawing because GLPick_FBO use
 * its own shaders to draw the object
 *
*/
class GlPick_FBO {
public:

    /// (width, height) = resolution of the current viewport
    GlPick_FBO(int width, int height);

    ~GlPick_FBO();

    void begin(int x, int y, GLfloat* mvp_mat);

    /// @return the picked primitive or -1
    int end();

    void set_name(unsigned id);

    /// Resize the FBO
    void resize(int w, int h);

private:
    GlFbo*   _fbo;       ///< Fbo used to render primitives to pick
    GlTex2D* _color_tex; ///< Color texture attached to the FBO
    GlTex2D* _depth_tex; ///< Depth texture attached to the FBO

    int _x, _y;          ///< pick position set with begin().
    bool _is_pick_init;  ///< Wether we're inside a begin() end()
};

#endif // GLPICK_FBO_HPP__

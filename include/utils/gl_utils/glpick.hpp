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
#ifndef GL_PICK_HPP__
#define GL_PICK_HPP__

#include "port_glew.h"
#include <vector>

/**
    @class GLPick
    @brief Utility to do legacy OpenGL 2.1 picking for object selection

    @warning This method of picking is extremely slow use it only for few
    (1000 < ) triangles/lines otherwise it is highly recommended to use
    FBO picking

    @warning This method of picking will ignore opengl lines and points
    thickness!

    Example of use :
    @code
        GLPick pick(NB_MAX_OBJECT);
        GLfloat m[16];
        glGetFloatv(GL_PROJECTION_MATRIX, m);

        pick.begin(m, mouse_x, mouse_y);
        {
            // Associate an integer with the next primitive drawing
            pick.set_name(0);
            drawFirstObject();

            pick.set_name(1);
            drawFirstObject();
        }
        // retrieve the integer associated to th selected object
        unsigned name = pick.end();
    @endcode
*/
class GLPick {
public:

    /// @param nb_hit_max maximum number of primitives to be drawn.
    /// @param
    GLPick(int nb_hit_max) :
        _pick_size(3.0f),
        _is_pick_init(false),
        _hit_buffer(nb_hit_max*2)
    {    }

    GLPick() :
        _pick_size(3.0f),
        _is_pick_init(false),
        _hit_buffer(1024)
    {    }

    /// @param nb_hit_max maximum number of primitives to be drawn.
    void set_nb_hit_max(int max) { _hit_buffer.resize(max*2); }

    /// Switch to picking mode. No primitive will be drawn to the screen after
    /// this call
    /// @param proj_mat the opengl projection matrix used to pick the objects.
    /// Usually we want the same as what's drawn, so a glGet() of the current
    /// projection matrix should be enough
    /// @param x : x position of the mouse
    /// @param y : y position of the mouse
    /// @warning imbricated begin() and() are not allowed
    /// @see end()
    void begin(const GLfloat* proj_mat, GLfloat x, GLfloat y);

    /// @return return the nearest primitive picked or (-1) if none has been
    /// selected
    int end();

    /// set the current picking name to the primitives that are to be drawn
    /// after this call
    void set_name(unsigned name){ if(_is_pick_init) glLoadName(name); }

    /// Is picking activated ?
    bool is_pick_init() const { return _is_pick_init; }

    /// Get the position in world coordinates for the nearest picked point
    void world_picked_pos(GLfloat* p) const {
        p[0] = _world_picked_pos[0];
        p[1] = _world_picked_pos[1];
        p[2] = _world_picked_pos[2];
    }

    /// Get the position in eye coordinates for the nearest picked point
    void eye_picked_pos(GLfloat* p) const {
        p[0] = _eye_picked_pos[0];
        p[1] = _eye_picked_pos[1];
        p[2] = _eye_picked_pos[2];
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    /// width and height of the picking region under the mouse
    float _pick_size;

private:
    /// Are we between an begin() end() calls.
    bool _is_pick_init;

    /// In this class we suppose that the stack of names size will be always 1
    /// so stack_size must be equal to one. And the field corresponding to
    /// the name stack can be represented by a single unsigned called 'name'.
    struct Hit {
        unsigned stack_size; ///< intended to be always 1
        unsigned z_min;      ///< minimal z depth for the hit
        unsigned z_max;      ///< maximal z depth for the hit
        unsigned name;       ///< name of the hit
    };

    std::vector<Hit> _hit_buffer;

    /// eye coordinates of the nearest primitive picked
    GLfloat _eye_picked_pos[3];
    /// eye coordinates of the nearest primitive picked
    GLfloat _world_picked_pos[3];
};



#endif // GL_PICK_HPP__

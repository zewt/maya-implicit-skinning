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
#ifndef OPENGL_STUFF_HPP__
#define OPENGL_STUFF_HPP__

#include "port_glew.h"

/** Various OpenGL functions
 */

/// Enable the program used to display a textured quad
void EnableProgram();
/// Disable that program
void DisableProgram();

/// Initialize various OpenGL states, buffer objects, textures...
void init_opengl();

/// Draw a quad to paint on
void draw_quad();

/// Display the representation of a blending operatoron the screen at (x,y)
void draw_operator(int x, int y, int w, int h);

/// Draw the controller at position (x,y) in a w x h frame
void draw_global_controller(int x, int y, int w, int h);

GLuint init_tex_operator( int width, int height );

void load_shaders();

void erase_shaders();

/// Draw a circle on screen at position (x, y)  of radius rad
void draw_circle(int width, int height, int x, int y, float rad);

#endif // OPENGL_STUFF_HPP__

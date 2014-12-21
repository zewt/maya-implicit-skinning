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
#include "globals.hpp"
#include "cuda_gl_interop_wrapper.hpp"
#include "opengl_stuff.hpp"
#include "cuda_ctrl.hpp"
#include "blending_env_tex_interface.hpp"

#include <sstream>

void init_opengl()
{
    printf("\n --- OPENGL INFOS ---\n");
    printf("Implementation vendor : %s\n", glGetString(GL_VENDOR));
    printf("Renderer : %s\n", glGetString(GL_RENDERER));
    printf("Opengl version : %s\n", glGetString(GL_VERSION));
    printf("Shader version : %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    //printf("Supported extensions : %s\n", glGetString(GL_EXTENSIONS));
    int n;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n);
    printf("maximum attributes per vertex : %d\n", n);
    printf("\n --- END OPENGL INFOS ---\n");
    printf("\n");
    fflush(stdout);
}

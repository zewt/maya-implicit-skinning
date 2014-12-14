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
#include "glu_utils.hpp"

#include "port_glew.h"

// =============================================================================
namespace Glu_utils {
// =============================================================================


Vec3_cu project(Vec3_cu p)
{
    GLdouble mv      [16];
    GLdouble proj    [16];
    GLint    viewport[ 4];
    glGetIntegerv(GL_VIEWPORT        ,  viewport  );
    glGetDoublev (GL_MODELVIEW_MATRIX,  mv        );
    glGetDoublev (GL_PROJECTION_MATRIX, proj      );
    GLdouble x, y, z;
    gluProject(p.x, p.y, p.z, mv, proj, viewport, &x, &y, &z);
    return Vec3_cu( (float)x, (float)y, (float)z);
}

// -----------------------------------------------------------------------------

Vec3_cu un_project(Vec3_cu p)
{
    GLdouble mv      [16];
    GLdouble proj    [16];
    GLint    viewport[ 4];
    glGetIntegerv(GL_VIEWPORT        ,  viewport  );
    glGetDoublev (GL_MODELVIEW_MATRIX,  mv        );
    glGetDoublev (GL_PROJECTION_MATRIX, proj      );
    GLdouble x, y, z;
    gluUnProject(p.x, p.y, p.z, mv, proj, viewport, &x, &y, &z);
    return Vec3_cu( (float)x, (float)y, (float)z);
}


}// End GLU_UTILS ==============================================================




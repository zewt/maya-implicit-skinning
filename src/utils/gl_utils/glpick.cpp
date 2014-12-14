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
#include "glpick.hpp"

#include "glassert.h"
#include <cassert>

// -----------------------------------------------------------------------------

void GLPick::begin(const GLfloat* proj_mat, GLfloat x, GLfloat y)
{
    assert(!_is_pick_init);
    _eye_picked_pos[0] = x; _eye_picked_pos[1] = y;
    // init buffer for picking
    glAssert( glSelectBuffer(_hit_buffer.size() * 4, (GLuint*)&(_hit_buffer[0])) );
    // get viewport info
    GLint view[4];
    glAssert( glGetIntegerv(GL_VIEWPORT, view) );
    // set selection mode on
    glAssert( glRenderMode(GL_SELECT) );
    // init names stack
    glAssert( glInitNames() );
    // Push a dummy name to be able to pop/push an element with glLoadName()
    glAssert( glPushName(0) );
    // modify the viewing volume => tune here ?
    glAssert( glMatrixMode(GL_PROJECTION) );
    glAssert( glPushMatrix() );
    glAssert( glLoadIdentity() );
    glAssert( gluPickMatrix(x, y, _pick_size, _pick_size, view) );
    glAssert( glMultMatrixf(proj_mat) );

    glAssert( glMatrixMode(GL_MODELVIEW) );
    glAssert( glPushMatrix() );
    glAssert( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );
    _is_pick_init = true;
}

// -----------------------------------------------------------------------------

int GLPick::end()
{
    glFlush();
    assert(_is_pick_init);
    _is_pick_init = false;

    // Retreive matrices
    GLdouble mv      [16];
    GLdouble proj    [16];
    GLint    viewport[ 4];
    glGetIntegerv(GL_VIEWPORT        ,  viewport  );
    glGetDoublev (GL_MODELVIEW_MATRIX,  mv        );
    glGetDoublev (GL_PROJECTION_MATRIX, proj      );

    glAssert( glPopName() );
    // restore from modifying viewing volume
    glAssert( glMatrixMode(GL_PROJECTION) );
    glAssert( glPopMatrix() );
    glAssert( glMatrixMode(GL_MODELVIEW) );
    glAssert( glPopMatrix() );


    // retreive number of hits
    GLint hits = glRenderMode(GL_RENDER);
    GL_CHECK_ERRORS();
    assert(hits <= (int)_hit_buffer.size()); // check for buffer offerflow

    // retrieve selected name
    int      picked = (hits > 0) ? _hit_buffer[0].name  : -1;
    unsigned min_z  = (hits > 0) ? _hit_buffer[0].z_min :  0;
    for( int i = 1; i < hits; i++ )
    {
        const unsigned z = _hit_buffer[i].z_min;
        if ( z < min_z )
        {
            min_z  = z;
            picked = _hit_buffer[i].name;
        }
    }

    // Compute the z position of the picked joint in float
    // cf. Opengl doc of glSelectBuffer()
    _eye_picked_pos[2] = ((GLfloat)min_z) / (GLfloat)(0xffffffff);
    // Convert it into world coordinates
    GLdouble x, y, z;
    gluUnProject(_eye_picked_pos[0], _eye_picked_pos[1], _eye_picked_pos[2],
                 mv, proj, viewport,
                 &x, &y, &z);

    _world_picked_pos[0] = (GLfloat)x;
    _world_picked_pos[1] = (GLfloat)y;
    _world_picked_pos[2] = (GLfloat)z;

    return picked;
}

// -----------------------------------------------------------------------------

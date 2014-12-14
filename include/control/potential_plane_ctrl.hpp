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
#ifndef POTENTIAL_PLANE_CTRL_HPP__
#define POTENTIAL_PLANE_CTRL_HPP__

#include "vec3_cu.hpp"
#include "glsave.hpp"

class Potential_plane_ctrl {
public:
    Potential_plane_ctrl() : _setup(false)
    {
        _normal.set(0.f, 1.f, 0.f);
        _org.set(0.f, 0.f, 0.f);
    }

    void draw()
    {
        if(_setup){
            draw_plane(2000.f);
        }
    }

    bool       _setup;
    Vec3_cu _normal;
    Vec3_cu _org;

//private:
    void draw_plane(float size)
    {
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();

            GLfloat newR[16];
            Vec3_cu tmp = _normal;
            if(tmp.x != tmp.y || tmp.y != tmp.z || tmp.x != tmp.z)
                tmp = tmp + 1.f;
            else
                tmp = Vec3_cu(1.f,0.f,0.f);

            Vec3_cu Rz = tmp.cross(_normal);
            Vec3_cu Rx = _normal.cross(Rz);
            Rz.normalize(); _normal.normalize(); Rx.normalize();

            newR[0] = Rx.x; newR[4] = _normal.x; newR[ 8] = Rz.x; newR[12] = _org.x;
            newR[1] = Rx.y; newR[5] = _normal.y; newR[ 9] = Rz.y; newR[13] = _org.y;
            newR[2] = Rx.z; newR[6] = _normal.z; newR[10] = Rz.z; newR[14] = _org.z;
            newR[3] = 0.f ; newR[7] = 0.f;       newR[11] = 0.f ; newR[15] = 1.f;

            glMultMatrixf(newR);

            //GLEnabledSave save_light(GL_LIGHTING, true, true);
            glColor4f(1.f, 0.f, 0.f, 0.99f);
            glBegin (GL_QUADS);
            glNormal3f(0.f, 1.f, 0.f);
            glVertex3f(-(size/2.f), 0.f, -(size/2.f));
            glVertex3f(-(size/2.f), 0.f,  (size/2.f));
            glVertex3f( (size/2.f), 0.f,  (size/2.f));
            glVertex3f( (size/2.f), 0.f, -(size/2.f));
            glEnd();

            glPopMatrix();
        }
};

#endif // POTENTIAL_PLANE_CTRL_HPP__


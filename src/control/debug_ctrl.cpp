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
#include "debug_ctrl.hpp"

#include "globals.hpp"

#include "cuda_utils.hpp"

// -----------------------------------------------------------------------------

void Debug_ctrl::draw_gradient(const std::vector<int>& selected_points,
                               const Vec3_cu* d_gradient)
{
    Vec3_cu* vert = 0;

    g_mesh->_vbo.map_to(vert, GL_READ_ONLY);

    glBegin(GL_LINES);
    glColor4f(0.f, 0.f, 1.f, 1.f);
    for(unsigned i = 0; i<selected_points.size(); i++)
    {
        Vec3_cu n;
        Cuda_utils::mem_cpy_dth(&n, d_gradient + selected_points[i], 1);

        n.normalize();
        n = n * -5.f;
        const Mesh::Packed_data d = g_mesh->get_packed_vert_map()[selected_points[i]];

        Vec3_cu v = vert[ d.idx_data_unpacked ];

        glVertex3f(v.x      , v.y      , v.z      );
        glVertex3f(v.x + n.x, v.y + n.y, v.z + n.z);

    }
    glEnd();

    g_mesh->_vbo.unmap();
}

// -----------------------------------------------------------------------------

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
#include "loader_skel.hpp"

// =============================================================================
namespace Loader {
// =============================================================================

void compute_bone_lengths(Abs_skeleton& skel)
{
    int nb_bones = skel._bones.size();
    for(int bone_id = 0; bone_id < nb_bones; bone_id++)
    {
        int nb_sons = skel._sons[bone_id].size();
        float mean_len = 0.f;
        for(int s = 0; s < nb_sons; s++)
        {
            int son_id = skel._sons[bone_id][s];

            Vec3 p0 = skel._bones[bone_id]._frame.get_translation();
            Vec3 p1 = skel._bones[son_id ]._frame.get_translation();

            mean_len += (p0-p1).norm();
        }

        float len = nb_sons > 1 ? mean_len / (float)nb_sons : mean_len;
        skel._bones[bone_id]._length = len;
    }
}

} // END LOADER NAMESPACE ======================================================

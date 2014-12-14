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
#include "SKIN/main_window_skin.hpp"

// -----------------------------------------------------------------------------

void Main_window_skin::enable_animesh(bool state)
{
    toolBar->_wgt_fit->setEnabled( state );
    toolBoxMenu->setItemEnabled(5 /* tab debug     */, state);
    toolBoxMenu->setItemEnabled(4 /* tab bone edit */, state);
    toolBoxMenu->setItemEnabled(3 /* tab blending  */, state);
    toolBoxMenu->setItemEnabled(2 /* tab animation */, state);
    box_animesh_color->setEnabled( state );
    box_skeleton->setEnabled( state );
}

// -----------------------------------------------------------------------------

void Main_window_skin::enable_mesh(bool state)
{
    box_mesh_color->setEnabled( state );
}

// -----------------------------------------------------------------------------

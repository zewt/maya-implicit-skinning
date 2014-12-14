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
#include "color_ctrl.hpp"

#include "mesh.hpp"
extern Mesh* g_mesh;

#include "path_ctrl.hpp"
namespace Cuda_ctrl{
    extern Path_ctrl _paths;
}

#include "class_saver.hpp"

// -----------------------------------------------------------------------------

void Color_ctrl::load_class_from_file()
{
    std::string path = Cuda_ctrl::_paths._configuration;
    path += "config_colors.dat";
    load_class(this, path);
}


// -----------------------------------------------------------------------------

void Color_ctrl::save_class_to_file()
{
    std::string path = Cuda_ctrl::_paths._configuration;
    path += "config_colors.dat";
    save_class(this, path);
}

// -----------------------------------------------------------------------------

Color Color_ctrl::get(int enum_field){
    return _list[enum_field];
}

// -----------------------------------------------------------------------------

void  Color_ctrl::set(int enum_field, const Color& cl){
    _list[enum_field] = cl;

    if(enum_field == MESH_POINTS && g_mesh != 0)
        g_mesh->set_point_color_bo(cl.r, cl.g, cl.b, cl.a);
    save_class_to_file();
}

// -----------------------------------------------------------------------------

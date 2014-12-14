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
#include "loader.hpp"

#include "fbx_loader.hpp"
#include "obj_loader.hpp"
#include "off_loader.hpp"
#include "graph_loader.hpp"
#include "std_utils.hpp"

static std::string get_file_path(const std::string& path)
{
    std::string res;
    unsigned pos = path.find_last_of('/');

    if( (pos+1) == path.size())
        return path;
    else
    {
        if( pos != std::string::npos) res = path.substr(0, pos+1);
        else                          res = "";
    }

    return res;
}

// =============================================================================
namespace Loader {
// =============================================================================

Base_loader* make_loader(const std::string& file_name)
{
    std::string ext = Std_utils::to_lower( Std_utils::file_ext(file_name) );

    if( ext == ".fbx")
        return new Fbx_loader::Fbx_file(file_name);
    else if( ext == ".obj")
        return new Obj_loader::Obj_file(file_name);
    else if( ext == ".off")
        return new Off_loader::Off_file(file_name);
    else if( ext == ".skel")
        return new Graph_loader::Graph_file(file_name);
    else
        return 0;
}

// CLASS Base_loader ===========================================================

Base_loader::Base_loader(const std::string& file_name){
    _file_path = file_name;
    _path      = get_file_path(file_name);
}

//------------------------------------------------------------------------------

bool Base_loader::load_file(const std::string& file_name)
{
    _file_path = file_name;
    _path      = get_file_path(file_name);
    return false;
}

//------------------------------------------------------------------------------

bool Base_loader::save_file(const std::string& file_name)
{
    _file_path = file_name;
    _path      = get_file_path(file_name);
    return false;
}

// =============================================================================
} // namespace Loader
// =============================================================================

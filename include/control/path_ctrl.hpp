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
#ifndef PATH_CTRL_HPP__
#define PATH_CTRL_HPP__

#include <string>

/** @class Path_ctrl
    @brief This class holds various paths used into the application
*/
class Path_ctrl {
public:

    Path_ctrl() :
        _meshes("./resource/meshes/"),
        _textures("resource/textures/"),
        _screenshots("./resource/"),
        _configuration("./resource/app_config/")
    {  }

    std::string _meshes;
    std::string _textures;
    std::string _screenshots;
    std::string _configuration;
};

#endif // PATH_CTRL_HPP__

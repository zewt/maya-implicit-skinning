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
#ifndef OFF_LOADER_HPP__
#define OFF_LOADER_HPP__

#include "loader.hpp"

// =============================================================================
namespace Off_loader {
// =============================================================================

class Off_file : public Loader::Base_loader {
public:
    Off_file(const std::string& file_name) : Base_loader( file_name )
    { import_file(file_name);  }

    /// The loader type
    Loader::Loader_t type() const { return Loader::OFF; }

    bool import_file(const std::string& file_path);
    bool export_file(const std::string& file_path);

    /// off files have no animation frame
    void get_anims(std::vector<Loader::Base_anim_eval*>& anims) const { anims.clear(); }

    /// transform internal representation into generic representation
    /// which are the same here.
    void get_mesh(Loader::Abs_mesh& mesh) const {
        mesh.clear();
        mesh = _mesh;
    }

private:
    Loader::Abs_mesh _mesh;
};

}// END Off_loader =============================================================


#endif //OFF_LOADER_HPP__

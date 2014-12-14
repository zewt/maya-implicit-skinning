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
#ifndef GENERIC_LOADER_HPP__
#define GENERIC_LOADER_HPP__

#include "loader.hpp"

// =============================================================================
namespace Loader {
// =============================================================================

/**
 * @class Generic_file
 * @brief Generic file loader. Use it for automatic deletion of the loader
 *
 */
class Generic_file {
public:
    /// Create and parse file
    Generic_file(const std::string& file_name) {
        _file = make_loader(file_name);
    }

    ~Generic_file(){ delete _file; }

    Loader_t type() const { return _file->type(); }

    bool import_file(const std::string& file_path){
        delete _file;
        _file = make_loader(file_path);
        return _file != 0;
    }

    bool export_file(const std::string& file_path){
        return _file->export_file( file_path );
    }

    /// @return the type of objects that have been loaded
    EObj::Flags fill_scene(Scene_tree& tree, EObj::Flags flags = 0){
        return _file->fill_scene(tree, flags);
    }

    void get_anims(std::vector<Loader::Base_anim_eval*>& anims) const {
        return _file->get_anims(anims);
    }

    /// @return if the file format is supported
    bool supported() const { return _file != 0; }

    std::string file_path() const { return _file->_file_path; }

private:
    Base_loader* _file;
};

} // END LOADER NAMESPACE ======================================================

#endif // GENERIC_LOADER_HPP__

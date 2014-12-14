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
#ifndef LOADER_HPP
#define LOADER_HPP

#include <string>
#include <vector>
#include "transfo.hpp"
#include "loader_enum.hpp"
#include "loader_mesh.hpp"
#include "loader_skel.hpp"
#include "loader_anims.hpp"

/**
  @namespace Loader
  @brief this namespace holds the intermediate representation for file loading

  Because Files representation are versatiles we choose to represent in the same
  intermediate data structure the output of a file loader. Hence every loader
  of this project (fbx, obj, off, Collada etc.) will have to parse the files and
  convert to this representation. This way the copy to the real project
  representation will be the same for every file formats.

  The representation is extremly simplified: triangular mesh. Only one instance
  of a mesh/skeleton skin can be loaded. If a file contains more instances they
  will be ignored or concatenated for meshes.

*/

// =============================================================================
namespace Loader {
// =============================================================================

/// @class Base_loader
/// @brief Abstract class for file loading
class Base_loader {
protected:
    Base_loader() {}
    Base_loader(const std::string& file_name);
public:

    virtual ~Base_loader(){ }

    /// The loader type
    virtual Loader_t type() const { return NOT_HANDLE; }

    virtual bool import_file(const std::string& file_path) = 0;
    virtual bool export_file(const std::string& file_path) = 0;

    /// @param anims parsed animations
    virtual void get_anims(std::vector<Base_anim_eval*>& anims) const = 0;


    // @{ -----------------------------------------------------------------------------------------------------------
    // TODO: to be deleted:
    /// parse and load the file into memory
    /// @return true if succeed
    virtual bool load_file(const std::string& file_name);

    /// save data to disk
    /// @return true if succeed
    virtual bool save_file(const std::string& file_name);
    // @} -----------------------------------------------------------------------------------------------------------

    /// @name attributes
    /// the name of the file last loaded is saved internally since the
    /// name may be needed to find a path to a material or texture file.
    std::string _file_path; ///< Path + file name
    std::string _path;      ///< Path without file name
};

//------------------------------------------------------------------------------

/// Factory method
/// Allocate the correct loader given the file type and parse the file
/// @note supported formats : .obj, .off, .skel and .fbx
Base_loader* make_loader(const std::string& file_name);

} // END namespace Loader ======================================================

#endif // LOADER_HPP

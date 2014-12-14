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
#ifndef FBX_LOADER_HPP__
#define FBX_LOADER_HPP__

#include "loader.hpp"

#include <map>

/**
  @namespace Fbx_loader
  @brief Holds data structure and utilities to store and parse en FBX file

  This module use the FBX SDK to parse and store meshes/skeleton/skin/scenes.
  The SDK requires a global initialization at the programm startup. This
  is done with Fbx_loader::init(). Releasing memory of the SDK is done with
  Fbx_loader::clean().

  A specialization of Loader::Base_loader is provided to import/export FBX files
  into an intermediate data representation.

  @see Loader Fbx_loader::init() Fbx_loader::clean()
*/

// Forward def to avoid the depencie of FBX SDK in this header
namespace fbxsdk_2012_2 {
class KFbxNode;
class KFbxScene;
}

// =============================================================================
namespace Fbx_loader {
// =============================================================================

/// Initialize the fbx SDK memory manager. This is mandatory to use the SDK
/// and must be done once at the programm startup
/// @see clean()
void init();

/// This erase the fbx SDK memory manager and must be called once when the
/// application is closed
void clean();

/**
    @class Fbx_file
    @brief main interface for an alias fbx file.
*/
class Fbx_file : public Loader::Base_loader {
public:

    //friend class Fbx_anim_eval;

    Fbx_file (const std::string& file_name) :
        Loader::Base_loader(file_name),
        _fbx_scene(0)
    {
        load_file(file_name);
    }

    ~Fbx_file(){ free_mem(); }


    // ----------------------------------------------------------------------------------------------------------
    // TODO: implement these:
    bool import_file(const std::string& file_path){ return load_file(file_path); }
    bool export_file(const std::string& file_path){ return save_file(file_path); }

    /// @return parsed animations or NULL.
    void get_anims(std::vector<Loader::Base_anim_eval*>& anims) const {
        get_animations(anims);
    }
    // ----------------------------------------------------------------------------------------------------------


    bool load_file(const std::string& file_name);
    bool save_file(const std::string& file_name){
        Base_loader::load_file(file_name);
        // if(_fbx_scene == 0) return false;
        return false;
    }

    /// Transform internal FBX representation into our skeleton representation
    void get_skeleton(Loader::Abs_skeleton& skel) const;

    /// Get fbx animation evaluator. Animation are concatenated in 'anims'
    void get_animations(std::vector<Loader::Base_anim_eval*>& anims) const;

    /// transform internal fbx representation into generic representation
    void get_mesh(Loader::Abs_mesh& mesh);

    /// transform generic representation into internal fbx representation
    void set_mesh(const Loader::Abs_mesh& mesh);

    void free_mem();

private:
    /// compute attributes '_offset_verts' and '_size_mesh'
    void compute_size_mesh();

    /// The FBX file data once parsed with load_file()
    fbxsdk_2012_2::KFbxScene* _fbx_scene;

    /// Stores for each FBX mesh node the offset introduced in vertices index
    /// because we concatenate meshes
    std::map<const fbxsdk_2012_2::KFbxNode*, int> _offset_verts;

    /// Nb vertices of all the FBX meshes concatenated
    int _size_mesh;
};

}// END namespace FBX_LOADER ===================================================


#endif // FBX_LOADER_HPP__


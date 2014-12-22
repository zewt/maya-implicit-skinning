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
#ifndef FBX_UTILS_HPP__
#define FBX_UTILS_HPP__

#include <string>
// Note: if this fails in MSVC2013 with "Unsupported platform", you'll need to edit the
// kstring.h header and comment out the "KARCH_DEV_MSC <= 1600" bit at the top.  There's
// some deeply horrifying stuff going on in there.
#include <fbxsdk.h>
#include <fbxfilesdk/kfbxio/kfbxiosettings.h>
#include <fbxfilesdk/kfbxplugins/kfbxsdkmanager.h>
#include <fbxfilesdk/kfbxplugins/kfbxscene.h>
#include <fbxfilesdk/kfbxio/kfbximporter.h>
#include <fbxfilesdk/fbxfilesdk_nsuse.h>

#include "transfo.hpp"
#include "loader_mesh.hpp"

/**
 * @name Fbx_utils
 * @brief Utilities for the FBX SDK and interacting with our own datas.
 */
// =============================================================================
namespace Fbx_utils {
// =============================================================================

/// @param filename : file path and name to load
/// @param fbx_scene : the allocated scene by KFbxManager
/// @param g_manager : the global SDK manager initialized
/// @return If the scene has been succesfully loaded
bool load_scene(const std::string& filename,
                KFbxScene* fbx_scene,
                KFbxSdkManager* g_manager);

// -----------------------------------------------------------------------------
/// @name Handling the FBX tree data.
// -----------------------------------------------------------------------------

/// Find the first node of the attribute type attrib in the tree
KFbxNode* find_root(KFbxNode* root, KFbxNodeAttribute::EAttributeType attrib);

// -----------------------------------------------------------------------------
/// @name Printing various FBX informations
// -----------------------------------------------------------------------------

/// Print the node hierachy
void print_hierarchy(KFbxScene* pScene);

/// Print the list of anim stacks names in the given scene
void print_anim_stacks(KFbxScene* pScene);

// -----------------------------------------------------------------------------
/// @name Transformations
// -----------------------------------------------------------------------------

/// Function to get a node's global default position.
/// As a prerequisite, parent node's default local position must be already set.
void set_global_frame(KFbxNode* pNode, KFbxXMatrix pGlobalPosition);

/// Recursive function to get a node's global default position.
/// As a prerequisite, parent node's default local position must be already set.
KFbxXMatrix get_global_frame(const KFbxNode* pNode);

/// Get the geometry deformation local to a node.
/// It is never inherited by the children.
KFbxXMatrix geometry_transfo(KFbxNode* pNode);

// -----------------------------------------------------------------------------
/// @name Conversion FBX types to our types
// -----------------------------------------------------------------------------

/// copy 'd3' in array 't'
void copy(float* t[3], const fbxDouble3& d3);

/// FbxX matrix to our transformation type
Loader::CpuTransfo to_transfo(const KFbxXMatrix& mat);

/// Fbx matrix to our transformation type
Loader::CpuTransfo to_transfo(const KFbxMatrix& mat);

/// FBxVector4 to our loader vertex type
Loader::Vertex to_lvertex(const KFbxVector4& vec);

/// FBxVector4 to our loader normal type
Loader::Normal to_lnormal(const KFbxVector4& vec);

/// @return the string corresponding to the mapping mode
std::string to_string(KFbxGeometryElement::EMappingMode type);

/// @return the string corresponding to the attribute type
std::string to_string(KFbxNodeAttribute::EAttributeType type);

}// END NAMESPACE FBX_UTILS ====================================================

#endif // FBX_UTILS_HPP__

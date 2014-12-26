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
#ifndef BONE_TEX_HPP
#define BONE_TEX_HPP

#include "cuda_utils.hpp"

// Forward defs ----------------------------------------------------------------
struct HermiteRBF;
struct Cylinder;
struct Precomputed_prim;
class Bone_cu;
// End Forward defs ------------------------------------------------------------

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// Bones with a memory layout compatible with cuda textures
struct Bone_tex {
    /// @name Access directly with DBone_id
    /// @{
    Cuda_utils::HD_Array<int>              hd_bone_types;
    Cuda_utils::HD_Array<Bone_cu>          hd_bone;
    Cuda_utils::HD_Array<HermiteRBF>       hd_bone_hrbf;
    Cuda_utils::HD_Array<Precomputed_prim> hd_bone_precomputed;
    /// @}

    void resize(int nb_elt){
        hd_bone_types.      malloc( nb_elt );
        hd_bone.            malloc( nb_elt );
        hd_bone_hrbf.       malloc( nb_elt );
        hd_bone_precomputed.malloc( nb_elt );
    }

    void clear(){
        hd_bone_types.      erase();
        hd_bone.            erase();
        hd_bone_hrbf.       erase();
        hd_bone_precomputed.erase();
    }

    void update_device_mem(){
        hd_bone_types.      update_device_mem();
        hd_bone.            update_device_mem();
        hd_bone_hrbf.       update_device_mem();
        hd_bone_precomputed.update_device_mem();
    }

};

}// END SKELETON_ENV NAMESPACE ================================================


#endif // BONE_TEX_HPP

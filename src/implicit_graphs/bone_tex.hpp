#ifndef BONE_TEX_HPP
#define BONE_TEX_HPP

#include "cuda_utils.hpp"

// Forward defs ----------------------------------------------------------------
struct HermiteRBF;
struct Cylinder;
struct Precomputed_prim;
// End Forward defs ------------------------------------------------------------

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// Bones with a memory layout compatible with cuda textures
struct Bone_tex {
    /// @name Access directly with DBone_id
    /// @{
    Cuda_utils::HD_Array<int>              hd_bone_types;
    Cuda_utils::HD_Array<HermiteRBF>       hd_bone_hrbf;
    Cuda_utils::HD_Array<Precomputed_prim> hd_bone_precomputed;
    /// @}

    void resize(int nb_elt){
        hd_bone_types.      malloc( nb_elt );
        hd_bone_hrbf.       malloc( nb_elt );
        hd_bone_precomputed.malloc( nb_elt );
    }

    void clear(){
        hd_bone_types.      erase();
        hd_bone_hrbf.       erase();
        hd_bone_precomputed.erase();
    }

    void update_device_mem(){
        hd_bone_types.      update_device_mem();
        hd_bone_hrbf.       update_device_mem();
        hd_bone_precomputed.update_device_mem();
    }

};

}// END SKELETON_ENV NAMESPACE ================================================


#endif // BONE_TEX_HPP

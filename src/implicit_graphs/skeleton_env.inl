#include "joint_type.hpp"
#include "bone.hpp"
#include "blending_env.hpp"
#include "blending_functions.hpp"
#include "idx3_cu.hpp"

#include <vector>

// =============================================================================
namespace Skeleton_env {
// =============================================================================

IF_CUDA_DEVICE_HOST static inline
BBox_cu fetch_grid_bbox_and_res(Skel_id id, Vec3i_cu& res)
{
    #ifdef __CUDA_ARCH__
    float4 a = tex1Dfetch(tex_grid_bbox, id*2 + 0);
    float4 b = tex1Dfetch(tex_grid_bbox, id*2 + 1);
    #else
    float4 a = hd_grid_bbox[id*2 + 0];
    float4 b = hd_grid_bbox[id*2 + 1];
    #endif
    res.x = res.y = res.z = (int)a.w;
    return BBox_cu(a.x, a.y, a.z,
                   b.x, b.y, b.z);
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
int fetch_grid_offset(Skel_id id){
    #ifdef __CUDA_ARCH__
    int2 s = tex1Dfetch(tex_offset, id);
    return s.y;
    #else
    return hd_offset[id].grid_data;
    #endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
int fetch_grid(int offset, int cell_id){
    #ifdef __CUDA_ARCH__
    return tex1Dfetch(tex_grid, cell_id + offset);
    #else
    return hd_grid[cell_id + offset];
    #endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
Cluster_id fetch_grid_blending_list_offset(Skel_id id, const Vec3_cu& pos)
{
    Vec3i_cu res;
    BBox_cu bb = fetch_grid_bbox_and_res( id, res);
    // compute linear grid index
    Vec3i_cu i3 = bb.index_grid_cell( res, pos );
    Idx3_cu idx(res, i3);

    // test if in grid
    if( idx.is_out() ) return Cluster_id(-1);

    // retreive offset
    int offset = fetch_grid_offset(id);
    int ptr_list = fetch_grid(offset, idx.to_linear());

    return Cluster_id(ptr_list);
}

// -----------------------------------------------------------------------------

__device__ static inline
Cluster_cu fetch_grid_blending_list(Cluster_id cid)
{
    #ifdef __CUDA_ARCH__
    int4 s = tex1Dfetch(tex_grid_list, cid.id());
    return *reinterpret_cast<Cluster_cu*>(&s);
    #else
    return hd_grid_blending_list[ cid.id() ];
    #endif
}

// -----------------------------------------------------------------------------

__device__ static inline
Cluster_id fetch_blending_list_offset(Skel_id id){
    int2 s = tex1Dfetch(tex_offset, id);
    return Cluster_id( s.x );
}

// -----------------------------------------------------------------------------

__device__ static inline
Cluster_cu fetch_blending_list(Cluster_id cid)
{
    int4 s = tex1Dfetch(tex_blending_list, cid.id());
    return *reinterpret_cast<Cluster_cu*>(&s);
}

__device__ static inline
HermiteRBF fetch_bone_hrbf(DBone_id i)
{
    int internal = tex1Dfetch(tex_bone_hrbf, i.id());
    return *reinterpret_cast<HermiteRBF*>(&internal);
}

// -----------------------------------------------------------------------------

__device__ static inline
Precomputed_prim fetch_bone_precomputed(DBone_id i)
{
    int internal = tex1Dfetch(tex_bone_precomputed, i.id());
    return *reinterpret_cast<Precomputed_prim*>(&internal);
}

// -----------------------------------------------------------------------------

__device__ static inline
EBone::Bone_t fetch_bone_type(DBone_id bone_id)
{
    return (EBone::Bone_t)tex1Dfetch(tex_bone_type, bone_id.id());
}

// -----------------------------------------------------------------------------

__device__ static inline
float fetch_and_eval_bone(DBone_id bone_id, Vec3_cu& gf, const Point_cu& x)
{
    EBone::Bone_t bone_type = (EBone::Bone_t)tex1Dfetch(tex_bone_type, bone_id.id());

    if( bone_type == EBone::HRBF)
    {
        HermiteRBF hrbf = fetch_bone_hrbf(bone_id);
        return hrbf.fngf(gf, x);
    }
    else if( bone_type == EBone::PRECOMPUTED)
    {
        Precomputed_prim prim = fetch_bone_precomputed(bone_id);
        return prim.fngf(gf, x);
    }
    else // if(bone_type == Bone_type::SSD)
    {
        gf = Vec3_cu(0.f, 0.f, 0.f);
        return 0.f;
    }
}

// -----------------------------------------------------------------------------

__device__ static inline
float fetch_binop_and_blend(Vec3_cu& grad,
                            EJoint::Joint_t type,
                            Blending_env::Ctrl_id ctrl_id,
                            Cluster_id clus_id,
                            float f1, float f2,
                            const Vec3_cu& gf1, const Vec3_cu& gf2)
{
    if( type == EJoint::MAX)
    {
        #if 1
        return Umax::fngf(grad, f1, f2, gf1, gf2);
        #else
        return Circle_anim::fngf(grad, f1, f2, gf1, gf2);
        #endif
    }
    #ifndef FAST_COMPILE
    else if( type == EJoint::GC_ARC_CIRCLE_TWEAK)
    {
        #if 1
        const Dyn_circle_anim op(ctrl_id);
        return op.fngf(grad, f1, f2, gf1, gf2);
        #else
        return Uclean::fngf(grad, f1, f2, gf1, gf2);
        #endif
    }
    else if( type == EJoint::BULGE)
    {
        #if 1
        if( f1 > 0.55f && f2 > 0.55f)
            return Umax::fngf(grad, f1, f2, gf1, gf2);
        else
        {
            Blending_env::Op_id id;
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_D );
            return Dyn_Operator3D_cu(id, ctrl_id).fngf(f1, f2, gf1, gf2, grad);
        }

//        const /*Dyn_4D_bulge*/Dyn_Bulge_Closed_Hyperbola<1> op(ctrl_id/*, tex1Dfetch(tex_bulge_strength, clus_id.id()) */);
//        return op.fngf(grad, f1, f2, gf1, gf2);
        #else
        return /*Static_4D_bulge*//*BulgeInContact*/BulgeFreeBlending::fngf(grad, f1, f2, gf1, gf2);
        #endif
    } else {
        Blending_env::Op_id id = ((int)type) - ((int)EJoint::BEGIN_CUSTOM_OP_ID);
        return Dyn_Operator3D_cu(id, ctrl_id).fngf(f1, f2, gf1, gf2, grad);
    }
    #endif
}


}// END SKELETON_TEXTURE NAMESPACE =============================================

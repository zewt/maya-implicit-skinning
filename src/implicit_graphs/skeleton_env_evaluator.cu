#include "skeleton_env_evaluator.hpp"
#include "blending_functions.hpp"


/**
 *  @namespace Blend_func
 *  @brief Holds the blending functions used to blend the skeleton's bones
 */
// =============================================================================
namespace Blend_func {
// =============================================================================

/// @brief Blending between cluster pairs  @see Skeleton_env
//typedef Op_test BlendP;
typedef Umax Pairs;
//typedef Uclean BlendP;
//typedef Circle_anim BlendP;
//typedef OMUCircle BlendP;
//typedef Ucircle BlendP;
//typedef Ifrom<Umax> BlendP; // Umin
//typedef BulgeFreeBlending Pairs;
//typedef BulgeInContact BlendP;

/// @brief blending between cluster's bones
/// The blending of bones that belongs to the same cluster
/// (i.e. bones that are on the same level in the hierarchy and has the same
/// parent)
/// @see Skeleton_env
//typedef Umax Cluster;
typedef BulgeFreeBlending Cluster;
//typedef Uclean BlendG;
//typedef Usum BlendG;

}// END BlendingFunc ===========================================================

#define USE_GRID_ // Not compatible with had_hoc hand !

__device__ static
float eval_cluster(Vec3_cu& gf_clus, const Point_cu& p, int size, Skeleton_env::DBone_id first_bone)
{
    gf_clus = Vec3_cu(0.f, 0.f, 0.f);

    // if(size == 0) return 0.f; // This should be guaranted by construction

    float f      = 0.f;
    float f_clus = 0.f;
    Vec3_cu gf;
    for(int i = 0; i < size; i++){
        f = fetch_and_eval_bone(first_bone+i, gf, p);
        f_clus = Blend_func::Cluster::fngf(gf_clus, f_clus, f, gf_clus, gf);
    }
    return f_clus;
}

__device__ static inline
Skeleton_env::Cluster_cu fetch_blending_list_int(Skeleton_env::Cluster_id cid)
{
#ifndef USE_GRID_
    return fetch_blending_list(cid);
#else
    return fetch_grid_blending_list(cid);
#endif
}

__device__
float Skeleton_env::compute_potential(Skel_id skel_id, const Point_cu& p, Vec3_cu& gf)
{
    typedef Cluster_cu Clus;
    float f = 0.f;
    gf = Vec3_cu(1.f, 0.f, 0.f);

#ifndef USE_GRID_
    // Without space acceleration structure
    Cluster_id off_cid = fetch_blending_list_offset( skel_id );
#else
    // With a grid as acceleration structure
    Cluster_id off_cid = fetch_grid_blending_list_offset(skel_id, p.to_vector());
    if( !off_cid.is_valid() /*Means we are outside the skeleton bbox*/)
        return 0.f;
#endif

    // Clusters contains at first a list of pairs with dynamic blending
    // each pair is blend to others with a max then the rest of the skeleton
    // is blended  with a max
    Clus clus = fetch_blending_list_int( off_cid );

    // In the first cluster we don't store the blending type and controller id
    const int nb_pairs      = clus.nb_pairs;

    // Blend the pairs
    for(int i = 0; i < nb_pairs*2; i += 2)
    {
        bool first = true;
        float fn;
        Vec3_cu gfn;
        for(int j = 0; j < 2; j++){
            clus = fetch_blending_list_int( off_cid + i + j );
            if(clus.nb_bone == 0)
                continue;

            float xfn;
            Vec3_cu xgfn;
            xfn = eval_cluster(xgfn, p, clus.nb_bone, clus.first_bone);

            // If this is the first input that has any bones, just store it.  Otherwise, blend
            // it with the ones we have so far.
            if(first)
            {
                fn = xfn;
                gfn = xgfn;
                first = false;
            } else {
                // Blend the pair
                fn = fetch_binop_and_blend(gfn, clus.blend_type, clus.ctrl_id,  off_cid + i,
                        fn, xfn, gfn, xgfn); // FIXME: This call should be specific to grid when enabled
            }
        }

        // Blend with the other pairs
        f =  Blend_func::Pairs::fngf(gf, f, fn, gf, gfn);
    }

    return f;
}

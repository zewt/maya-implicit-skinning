#include "precomputed_prim.hpp"
#include "precomputed_prim_env.hpp"

void Precomputed_prim::initialize(){
    assert(_id < 0);
    _id = Precomputed_env::new_instance();
}

void Precomputed_prim::clear(){
    assert(_id >= 0);
    Precomputed_env::delete_instance(_id);
}
__host__
void Precomputed_prim::fill_grid_with(Skeleton_env::Skel_id skel_id, const Bone* bone)
{
    Precomputed_env::init_instance(_id, skel_id, bone);
}

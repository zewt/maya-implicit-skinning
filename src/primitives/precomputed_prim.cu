#include "precomputed_prim.hpp"
#include "bone.hpp"
#include "cuda_utils.hpp"
#include "transfo.hpp"
#include "bbox.hpp"
#include "animesh_kers.hpp"
#include "precomputed_prim_constants.hpp"
#include "point_cu.hpp"
#include "skeleton_env_type.hpp"

#include "hrbf_env_tex.hpp"

/// These header needs HRBF_Env to be included first
/// @{
#include "hermiteRBF.hpp"
#include "hermiteRBF.inl"
/// @}

#include "blending_env.hpp"
#include "skeleton_env.hpp"

#include <deque>
#include <iostream>

namespace { __device__ void fix_debug() { } }


using namespace Cuda_utils;

// All info for the object is stored here, instead of in the class itself, so the object
// remains just a single ID.  This is needed because other parts of the code expect to be
// able to store a Precomputed_prim in a texture, and it allows accessing the same data
// from device memory.
struct PrecomputedInfo
{
    __host__ PrecomputedInfo():
        id(-1),
        tex_grid(0),
        d_grid(NULL)
    {
    }
    int id;

    /// First float is the potential last three floats the gradient
    cudaTextureObject_t tex_grid;

    // Transformation associated to a grid in initial position.
    // point_in_grid_space = h_grid_transform[i] * point_in_world_space;
    Transfo grid_transform;

    /// Transformation set by the user for every grid.
    Transfo user_transform;

    /// Temporary buffer used to transfer 'grid_transform' rapidly to 'anim_transform'
    /// The buffer is filled with set_transform()
    /// grid_transfo_buffer = grid_transform * user_transform
    Transfo grid_transfo_buffer;

    Device::CuArray<float4> *d_grid;
};

std::vector<PrecomputedInfo> h_precomputed_info;
Device::Array<PrecomputedInfo> d_precomputed_info;
__device__ __managed__ const PrecomputedInfo *dp_precomputed_info;




namespace Precomputed_env{
using namespace Cuda_utils;

/// Give the transformation from world coordinates to the grid defined
/// by the bouding box 'bb' of resolution 'res'
static Transfo world_coord_to_grid(const OBBox_cu& obbox, int res)
{
    Vec3_cu v = obbox._bb.pmax - obbox._bb.pmin;
    float3 steps = {(float)res / v.x, (float)res / v.y, (float)res / v.z};

    Mat3_cu scale = Mat3_cu(steps.x, 0.f    , 0.f,
                            0.f    , steps.y, 0.f,
                            0.f    , 0.f    , steps.z);

    return Transfo::translate( Vec3_cu(0.5f, 0.5f, 0.5f) ) * // Because Cuda texture is going to be filtered (trilinear) we need to offset
           Transfo(scale) *                                  // Scale so the point coordinates matches the grid units lengths
           Transfo::translate( -(Vec3_cu)obbox._bb.pmin ) *  // translate to the grid origin
           obbox._tr.fast_invert();                          // To box coordinates
}


// We only ever need one of these at a time, so use a surface instead of a surface object.
surface<void, cudaSurfaceType3D> fill_surface;

/// @warning bone_id in device mem ! not the same as Skeleton class
/// @see Skeleton_Env::get_idx_device_bone()
__global__ static
void fill_grid_kernel(int grid_size,
                      Skeleton_env::DBone_id bone_id,
                      float3 steps,
                      int grid_res,
                      Point_cu org,
                      Transfo transfo)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < grid_size)
    {
        int x = idx % grid_res;
        int y = (idx / grid_res) % grid_res;
        int z = idx / (grid_res * grid_res);
        Point_cu off = Point_cu(steps.x * x, steps.y * y, steps.z * z);

        Point_cu p = org + off;

        Vec3_cu gf(0.f, 0.f, 0.f);

        HermiteRBF hrbf = Skeleton_env::fetch_bone_hrbf( bone_id );
        float pot = hrbf.fngf(gf, transfo * p);
        pot = pot < 0.00001f ? 0.f  : pot;

        float4 element = make_float4(gf.x, gf.y, gf.z, pot);
        surf3Dwrite(element, fill_surface, x*sizeof(float4), y, z, cudaBoundaryModeTrap);
    }
}

/// Filling a 3D grid with an hrbf primitive
/// @param bone_id bone to fill the grid with. Be aware that this id is not
/// the same as bone ids in Skeleton class. use Skeleton_Env::get_idx_device_bone()
/// to convert Skeleton's ids to device ids;
/// @param steps is the (x, y, z) length of each grid cell
/// @param grid_res is the number of cells in the (x, y, z) directions
/// @param org is the 3D coordinates of the origine of the grid.
/// @param transfo the transformation apllied to the implicit primitive which is
/// evaluated
/// @param d_out_grid the linear array to store the grid into.
/// @warning bone_id is the id in device memory ! It is not the same as the one
/// in Skeleton class use Skeleton_Env::get_idx_device_bone()
/// @see Skeleton_Env::get_idx_device_bone()
void fill_grid_with_fngf(PrecomputedInfo &info,
                         Skeleton_env::DBone_id device_bone_id,
                         float3 steps,
                         int grid_res,
                         Point_cu org,
                         Transfo transfo,
                         int grids,
                         int blocks)
{
    Skeleton_env::bind();
    HRBF_env::bind_local();

    cudaBindSurfaceToArray(fill_surface, info.d_grid->getCudaArray());
    CUDA_CHECK_ERRORS();

    fill_grid_kernel<<<grids, blocks>>>
    (info.d_grid->size(), device_bone_id, steps, grid_res, org, transfo);
    CUDA_CHECK_ERRORS();

    Skeleton_env::unbind();
    HRBF_env::unbind_local();
}

static void fill_grid(PrecomputedInfo &info,
                      Bone::Id bone_id,
                      Skeleton_env::Skel_id skel_id,
                      const OBBox_cu& obbox,
                      int res)
{
    assert(GRID_RES_3 == info.d_grid->size());

    Vec3_cu lengths = obbox._bb.lengths();
    float3  steps = {lengths.x / (float)res,
                     lengths.y / (float)res,
                     lengths.z / (float)res};

    
    const int ker_block_size = 64;
    const int ker_grid_size  =
            (info.d_grid->size() + ker_block_size - 1) / ker_block_size;


    if(ker_grid_size > 65535){
        std::cerr << "ERROR: The grid is too large cuda thread size is exceeded." << std::endl;
        assert(false);
    }

    Skeleton_env::DBone_id device_bone_id = Skeleton_env::bone_hidx_to_didx(skel_id, bone_id);

    fill_grid_with_fngf(info,
                        device_bone_id,
                        steps,
                        res,
                        obbox._bb.pmin,
                        obbox._tr,
                        ker_grid_size,
                        ker_block_size);

    CUDA_CHECK_ERRORS();
}

__device__
float fetch_potential(const PrecomputedInfo &info, const Point_cu& p)
{
    Point_cu  r = info.grid_transfo_buffer * p;

//    if( is_in_grid(r) )
//    {
//        float4 res = tex3D(tex_grid, r.x, r.y, r.z);
//        return res.x;
//    }

    return 0.f;
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu fetch_gradient(const PrecomputedInfo &info, const Point_cu& p)
{
    Point_cu  r = info.grid_transfo_buffer * p;

//    if( is_in_grid(r) )
//    {
//        float4 res = tex3D(tex_grid, r.x, r.y, r.z);
//        return Vec3_cu(res.y, res.z, res.w);
//    }

    return Vec3_cu(0.f, 0.f, 0.f);
}

}

const Transfo& Precomputed_prim::get_user_transform() const
{
    return get_info().user_transform;
}

void Precomputed_prim::set_transform(const Transfo& transfo)
{
    PrecomputedInfo &info = get_info();
    info.user_transform = transfo;
    info.grid_transfo_buffer = info.grid_transform * info.user_transform.fast_invert();
}

void Precomputed_prim::update_device_transformations()
{
    for(int i = 0; i < (int) h_precomputed_info.size(); ++i)
        update_device(i);
}

void Precomputed_prim::initialize()
{
    using namespace Precomputed_env;
    assert(_id == -1);

    // find the first free element (which is represented by negative offsets)
    int idx = 0;
    for(; idx<h_precomputed_info.size(); idx++)
        if(h_precomputed_info[idx].id == -1) break;
    _id = idx;

    h_precomputed_info.resize(max((int) h_precomputed_info.size(), _id+1));
    PrecomputedInfo &info = h_precomputed_info[_id];

    info.id = _id;
    info.d_grid = new Device::CuArray<float4>();
    info.d_grid->set_cuda_flags(cudaArraySurfaceLoadStore);
    info.d_grid->malloc(GRID_RES, GRID_RES, GRID_RES);

    {
        cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeArray;
        resDesc.res.array.array = info.d_grid->getCudaArray();

        cudaTextureDesc tex;
        memset(&tex, 0, sizeof(tex));
        tex.normalizedCoords = false;
        tex.filterMode = cudaFilterModeLinear;
        tex.addressMode[0] = cudaAddressModeBorder;
        tex.addressMode[1] = cudaAddressModeBorder;
        tex.addressMode[2] = cudaAddressModeBorder;

        cudaCreateTextureObject(&info.tex_grid, &resDesc, &tex, NULL);
        CUDA_CHECK_ERRORS();
    }

    update_device(_id);
}

void Precomputed_prim::update_device(int _id) {
    // If every entry in h_precomputed_info is unused, erase it and d_precomputed_info.
    // If we don't do this then we'll never deallocate d_precomputed_info, which will cause
    // a hang if we're released after CUDA is deinitialized.
    bool any_in_use = false;
    for(int i = 0; i < (int) h_precomputed_info.size(); ++i)
        if(h_precomputed_info[i].id != -1)
            any_in_use = true;
    if(!any_in_use)
    {
        h_precomputed_info.clear();
        d_precomputed_info.erase();
        dp_precomputed_info = NULL;
        return;
    }

    d_precomputed_info.realloc((int) h_precomputed_info.size());
    dp_precomputed_info = d_precomputed_info.ptr();

    // Update the buffer in device memory.
    PrecomputedInfo &info = h_precomputed_info[_id];
    d_precomputed_info.copy_from(h_precomputed_info);
}

const PrecomputedInfo &Precomputed_prim::get_info() const {
    assert(_id >= 0);

#ifdef __CUDA_ARCH__
    const PrecomputedInfo &info = dp_precomputed_info[_id];
#else
    assert(_id < h_precomputed_info.size());
    const PrecomputedInfo &info = h_precomputed_info[_id];
#endif

    assert(info.id != -1);

    return info;
}

PrecomputedInfo &Precomputed_prim::get_info() {
    return const_cast<PrecomputedInfo &>(const_cast<const Precomputed_prim *>(this)->get_info());
}

void Precomputed_prim::clear() {
    using namespace Precomputed_env;

    PrecomputedInfo &info = get_info();
    if(info.tex_grid != 0)
    {
        cudaDestroyTextureObject(info.tex_grid);
        CUDA_CHECK_ERRORS();
    }
    info.tex_grid = 0;

    delete info.d_grid;
    info.d_grid = NULL;

    info.id = -1;
    int old_id = _id;
    _id = -1;

    // This isn't really necessary.  It avoids having stale info in device memory on entries
    // that should never be accessed, but it can help in debugging.
    update_device(old_id);
}

__host__
void Precomputed_prim::fill_grid_with(Skeleton_env::Skel_id skel_id, const Bone* bone)
{
    using namespace Precomputed_env;

    PrecomputedInfo &info = get_info();
    Bone::Id bone_id = bone->get_bone_id();
    OBBox_cu obbox = bone->get_obbox();

    // Compute the primive's grid
    fill_grid(info, bone_id, skel_id, obbox, GRID_RES);

    // Adding the transformation to evaluate the grid
    info.grid_transform = world_coord_to_grid(obbox, GRID_RES);
    info.user_transform = Transfo::identity();

    update_device(_id);
}

__device__
float Precomputed_prim::f(const Point_cu& x) const
{
    using namespace Precomputed_env;

    const PrecomputedInfo &info = get_info();
    return fetch_potential(info, x);
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu Precomputed_prim::gf(const Point_cu& x) const
{
    using namespace Precomputed_env;

    const PrecomputedInfo &info = get_info();
    return fetch_gradient(info, x);
}

// -----------------------------------------------------------------------------
__device__
bool is_in_grid(const Point_cu& pt)
{
    const float res = (float)GRID_RES;

    return pt.x >= 0.5        && pt.y >= 0.5f       && pt.z >= 0.5f &&
           pt.x <  res - 0.5f && pt.y <  res - 0.5f && pt.z <  res - 0.5f;
}

__device__
float Precomputed_prim::fngf(Vec3_cu& grad, const Point_cu& p) const
{
    using namespace Precomputed_env;

    const PrecomputedInfo &info = get_info();
    Point_cu  r = info.grid_transfo_buffer * p;

    // XXX: Can we avoid needing to check this using texture borders, since each grid is now in
    // a separate texture?
    if( !is_in_grid( r ) )
    {
        grad = Vec3_cu(0.f, 0.f, 0.f);
        return 0.f;
    }

    float4 res = tex3D<float4>(info.tex_grid, r.x, r.y, r.z);
    grad.x = res.x;
    grad.y = res.y;
    grad.z = res.z;

    grad = info.user_transform * grad;
    return res.w;
}

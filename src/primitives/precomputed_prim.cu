#include "precomputed_prim.hpp"
#include "bone.hpp"
#include "cuda_utils.hpp"
#include "transfo.hpp"
#include "bbox.hpp"
#include "animesh_kers.hpp"
#include "precomputed_prim_constants.hpp"
#include "point_cu.hpp"
#include "skeleton_env_type.hpp"

// XXX: cleanup

#include "hrbf_env_tex.hpp"

/// These header needs HRBF_Env to be included first
/// @{
#include "hermiteRBF.hpp"
#include "hermiteRBF.inl"
/// @}

#include "blending_env_tex.hpp"

/// This include needs blending_env_tex.hpp but we will not bind it and use it
#include "skeleton_env_tex.hpp"

#include <deque>
#include <iostream>



namespace { __device__ void fix_debug() { } }

// Forward def skeleton_env.hpp
#include "skeleton_env_type.hpp"
namespace Skeleton_env{
    extern DBone_id bone_hidx_to_didx(Skel_id skel_id, Bone::Id bone_hidx);
}



using namespace Cuda_utils;

struct PrecomputedInfo
{
    PrecomputedInfo():
        id(-1),
        allocated(false),
        tex_grids(0),
        tex_transform(0),
        tex_transform_grad(0),
        tex_offset(0)
    {
    }
    int id;

    bool allocated;
    /// First float is the potential last three floats the gradient
    cudaTextureObject_t tex_grids;
    cudaTextureObject_t tex_transform;
    cudaTextureObject_t tex_transform_grad;
    cudaTextureObject_t tex_offset;
};

std::vector<PrecomputedInfo> h_precomputed_info;
Device::Array<PrecomputedInfo> d_precomputed_info;
__device__ __managed__ const PrecomputedInfo *dp_precomputed_info;




namespace Precomputed_env{
using namespace Cuda_utils;


// XXX: This copies all precomputed grids into a single big texture.  We should probably have a
// separate d_block for each skeleton.


/// This array is updated by update_device_transformations()
/// Transformation of a 3d point to the texture coordinates of 'tex_grids'.
/// d_anim_transform[inst_id] = tex_grid_transfo
Device::Array<Transfo> d_anim_transform;
Device::Array<Transfo> d_grad_transform;

/// Temporary buffer used to transfer 'h_grid_transform' rapidly to 'd_anim_transform'
/// The buffer is filled with set_transform()
/// h_grid_transfo_buffer[i] = h_grid_transform[i] * h_user_transform[i]
Host::PL_Array<Transfo> h_grid_transfo_buffer;

/// Transformation associated to a grid in initial position.
/// point_in_grid_space = h_grid_transform[i] * point_in_world_space;
Host::Array<Transfo> h_grid_transform;

/// Transformation set by the user for every grid.
Host::Array<Transfo> h_user_transform;

/// In Cuda textures has a limited size of 2048^3 (considering a GPU with
/// compute capability 2.0). so for one texture we stack on the x, y and z axis
/// multiples grids. 'd_block' is a block made of multiples grids.
Device::CuArray<float4> d_block;

/// Array of grids : d_grids[inst_id][grid_element]
std::deque< DA_float4* > d_grids;

/// This array is used to look up the 'd_block' array in order to find the grid
/// attached to an instance identifier. d_offset[id].x/y/z coordinates represent
/// the indices inside the block 'd_block' for the grid of identifier 'id'.
/// @code
/// int x = d_offset[id].x * GRID_RES;
/// int y = d_offset[id].y * GRID_RES;
/// int z = d_offset[id].z * GRID_RES;
/// int i = x + y * MAX_TEX_LENGTH + z * MAX_TEX_LENGTH * MAX_TEX_LENGTH
/// d_block[i] // first element of the grid of indentifier 'id'
/// @endcode
/// GRID_SIZE defines the x, y and z length of the grid
/// @see Precomputed_Env::tex_grids
DA_int4 d_offset;
HA_int4 h_offset;

int nb_instances = 0;

void update_all_info()
{
    // XXX only need to do this if something has been reallocated, and only for that item
    for(int i = 0; i < (int) h_precomputed_info.size(); ++i)
    {
        Precomputed_prim::update_info(i);
    }
}

void clean_env()
{
    d_anim_transform.erase();
    d_grad_transform.erase();
    h_grid_transfo_buffer.erase();
    h_grid_transform.erase();
    h_user_transform.erase();
    d_block.erase();
    d_grids.clear();
    d_offset.erase();
    h_offset.erase();
    nb_instances = 0;
}

// -----------------------------------------------------------------------------

/// Fills the 'out_block' array with the grid 'in_grid'
__global__
void fill_block(DA_float4 in_grid, int3 org, int3 block_size, DA_float4 out_block)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < in_grid.size())
    {
        // Local (x, y, z) coordinates in the grid
        int3 grid = { idx % GRID_RES,
                     (idx / GRID_RES) % GRID_RES,
                      idx / (GRID_RES * GRID_RES) };

        // Coordinates in the block
        int3 block = {org.x + grid.x, org.y + grid.y, org.z + grid.z};
        // Convert 'block' to the linear indice in the array
        int idx_block =
                block.x +
                block.y * block_size.x +
                block.z * block_size.x * block_size.y;

        out_block[idx_block] = in_grid[idx];
    }
}

// -----------------------------------------------------------------------------

void copy_grids_to_cuarray_block()
{
    if(h_offset.size() == 0)
        return;

    int3 block_size = {0, 0, 0};
    for(int i = 0; i < h_offset.size(); i++)
    {
        if(h_offset[i].x < 0)
            continue;

        block_size.x = max(h_offset[i].x*GRID_RES + GRID_RES, block_size.x);
        block_size.y = max(h_offset[i].y*GRID_RES + GRID_RES, block_size.y);
        block_size.z = max(h_offset[i].z*GRID_RES + GRID_RES, block_size.z);
    }

    DA_float4 block_temp(block_size.x*block_size.y*block_size.z);

    // Looping through all grid's instances
    for(int i = 0; i < h_offset.size(); i++)
    {
        // (x, y, z) indices in the block :
        int4 off = h_offset[i];
        if(off.x < 0)
            continue;

        printf("copy_grids_to_cuarray_block %i, %i, size %i\n", i, h_offset[i].x, d_grids[i]->size());
        fflush(stdout);
        // Grid's origin (x, y, z) coordinates in the block
        int3 org = {off.x * GRID_RES, off.y * GRID_RES, off.z * GRID_RES };

        int ker_b_size = 64;
        int ker_g_size = ((*d_grids[i]).size() + ker_b_size - 1) / ker_b_size;

        // Filling all grid's element of instance 'i'
        fill_block<<<ker_g_size, ker_b_size >>>(*d_grids[i], org, block_size, block_temp);
    }

    d_block.malloc(block_size.x, block_size.y, block_size.z);
    d_block.copy_from(block_temp.ptr(), block_temp.size());
}

// -----------------------------------------------------------------------------

static void update_offset(int idx)
{
    const int block_length = MAX_TEX_LENGTH / GRID_RES;

    int4 grid_index = { idx % block_length,
                       (idx / block_length) % block_length,
                        idx / (block_length * block_length),
                        0 };

    h_offset[idx] = grid_index;
    d_offset.copy_from(h_offset);
}

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


// -----------------------------------------------------------------------------


/// @warning bone_id in device mem ! not the same as Skeleton class
/// @see Skeleton_Env::get_idx_device_bone()
__global__ static
void fill_grid_kernel(Skeleton_env::DBone_id bone_id,
                      float3 steps,
                      int grid_res,
                      Point_cu org,
                      Transfo transfo,
                      Cuda_utils::DA_float4 d_out_grid)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < d_out_grid.size())
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

        d_out_grid[idx] = make_float4(pot, gf.x, gf.y, gf.z);
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
void fill_grid_with_fngf(Skeleton_env::DBone_id device_bone_id,
                         float3 steps,
                         int grid_res,
                         Point_cu org,
                         Transfo transfo,
                         int grids,
                         int blocks,
                         Cuda_utils::DA_float4 d_out_grid)
{
    Skeleton_env::bind_local();
    HRBF_env::bind_local();

    fill_grid_kernel<<<grids, blocks >>>
    (device_bone_id, steps, grid_res, org, transfo, d_out_grid);

    Skeleton_env::unbind_local();
    HRBF_env::unbind_local();
}

static void fill_grid(Bone::Id bone_id,
                      Skeleton_env::Skel_id skel_id,
                      const OBBox_cu& obbox,
                      int res,
                      DA_float4& d_grid)
{
    assert(GRID_RES_3 == d_grid.size());

    Vec3_cu lengths = obbox._bb.lengths();
    float3  steps = {lengths.x / (float)res,
                     lengths.y / (float)res,
                     lengths.z / (float)res};

    const int ker_block_size = 64;
    const int ker_grid_size  =
            (d_grid.size() + ker_block_size - 1) / ker_block_size;


    if(ker_grid_size > 65535){
        std::cerr << "ERROR: The grid is too large cuda thread size is exceeded." << std::endl;
        assert(false);
    }

    using namespace Skeleton_env;

    DBone_id device_bone_id = bone_hidx_to_didx(skel_id, bone_id);

    fill_grid_with_fngf(device_bone_id,
                        steps,
                        res,
                        obbox._bb.pmin,
                        obbox._tr,
                        ker_grid_size,
                        ker_block_size,
                        d_grid);

    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

const Transfo& get_user_transform(int inst_id)
{
    return h_user_transform[inst_id];
}

// -----------------------------------------------------------------------------

void set_transform(int inst_id, const Transfo& transfo)
{
    h_grid_transfo_buffer[inst_id] = h_grid_transform[inst_id] * transfo.fast_invert();
    h_user_transform[inst_id] = transfo;
    d_grad_transform.set(inst_id, transfo);
}

// -----------------------------------------------------------------------------

void update_device_transformations()
{
    if(h_grid_transfo_buffer.size() > 0)
        d_anim_transform.copy_from(h_grid_transfo_buffer);
    Precomputed_env::update_all_info();
}

__device__
Transfo fetch_transform(const PrecomputedInfo &info)
{
    struct{
        float4 a;
        float4 b;
        float4 c;
        float4 d;
    } s;

    s.a = tex1Dfetch<float4>(info.tex_transform, info.id*4 + 0);
    s.b = tex1Dfetch<float4>(info.tex_transform, info.id*4 + 1);
    s.c = tex1Dfetch<float4>(info.tex_transform, info.id*4 + 2);
    s.d = tex1Dfetch<float4>(info.tex_transform, info.id*4 + 3);

    return *reinterpret_cast<Transfo*>(&s);
}

// -----------------------------------------------------------------------------

__device__
Transfo fetch_transform_grad(const PrecomputedInfo &info)
{
    struct{
        float4 a;
        float4 b;
        float4 c;
        float4 d;
    } s;

    s.a = tex1Dfetch<float4>(info.tex_transform_grad, info.id*4 + 0);
    s.b = tex1Dfetch<float4>(info.tex_transform_grad, info.id*4 + 1);
    s.c = tex1Dfetch<float4>(info.tex_transform_grad, info.id*4 + 2);
    s.d = tex1Dfetch<float4>(info.tex_transform_grad, info.id*4 + 3);

    return *reinterpret_cast<Transfo*>(&s);
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu fetch_offset(const PrecomputedInfo &info){
    int4 off = tex1Dfetch<int4>(info.tex_offset, info.id);
    return Vec3_cu(off.x*GRID_RES, off.y*GRID_RES, off.z*GRID_RES);
}

// -----------------------------------------------------------------------------

__device__
bool is_in_grid(const Point_cu& pt, Vec3_cu off)
{
    const float res = (float)GRID_RES;
    off = off + 0.5f;

    //plus/minus one are hacks because I forgot to pad
    return ((pt.x >= off.x + 1.f        ) & (pt.y >= off.y + 1.f        ) & (pt.z >= off.z + 1.f       ) &
            (pt.x <  (off.x + res - 1.f)) & (pt.y <  (off.y + res - 1.f)) & (pt.z <  (off.z + res - 1.f)));
}

// -----------------------------------------------------------------------------

/// @param p    the 3d texture coordinate of tex_grid
/// @param grad the gradient at point p
/// @return the potential at point p
__device__
float fetch_grid(const PrecomputedInfo &info, const Point_cu& p, Vec3_cu& grad)
{
    float4 res = tex3D<float4>(info.tex_grids, p.x, p.y, p.z);
    grad.x = res.y;
    grad.y = res.z;
    grad.z = res.w;
    return res.x;
}

__device__
float fetch_potential(const PrecomputedInfo &info, const Point_cu& p)
{
    Point_cu  r = fetch_transform(info) * p;
    Vec3_cu off = fetch_offset(info);

    r = r + off;

//    if( is_in_grid( r, off ) )
//    {
//        float4 res = tex3D(tex_grids, r.x, r.y, r.z);
//        return res.x;
//    }

    return 0.f;
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu fetch_gradient(const PrecomputedInfo &info, const Point_cu& p)
{
    Point_cu  r = fetch_transform(info) * p;
    Vec3_cu off = fetch_offset(info);

    r = r + off;

//    if( is_in_grid( r, off ) )
//    {
//        float4 res = tex3D(tex_grids, r.x, r.y, r.z);
//        return Vec3_cu(res.y, res.z, res.w);
//    }

    return Vec3_cu(0.f, 0.f, 0.f);
}

}

void Precomputed_prim::initialize()
{
    using namespace Precomputed_env;
    assert(_id < 0);

    // find the first free element (which is represented by negative offsets)
    int idx = 0;
    for(; idx<h_offset.size(); idx++)
        if( h_offset[idx].x < 0) break;

    // Add the instance
    if(idx == h_offset.size())
    {
        const int size = h_offset.size() + 1;
        h_offset.realloc( size );
        d_grids.push_back(new DA_float4(GRID_RES_3));
        d_offset.realloc( size );
        h_grid_transform.realloc(size);
        h_user_transform.realloc(size);
        h_grid_transfo_buffer.realloc(size);
        d_anim_transform.realloc(size);
        d_grad_transform.realloc(size);
    }
    else
        d_grids[idx] = new DA_float4(GRID_RES_3);

    update_offset(idx);

    nb_instances++;

    _id = idx;

    update_info(_id);
}

void Precomputed_prim::dealloc(int _id) {
    if(_id >= h_precomputed_info.size())
        return;

    PrecomputedInfo &info = h_precomputed_info[_id];
    if(!info.allocated)
        return;

    if(info.tex_transform != 0)
        cudaDestroyTextureObject(info.tex_transform);
    info.tex_transform = 0;
    if(info.tex_transform_grad != 0)
        cudaDestroyTextureObject(info.tex_transform_grad);
    info.tex_transform_grad = 0;
    if(info.tex_offset != 0)
        cudaDestroyTextureObject(info.tex_offset);
    info.tex_offset = 0;
    if(info.tex_grids != 0)
    {
        printf("dealloc texture object %i\n", info.tex_grids);
        cudaDestroyTextureObject(info.tex_grids);
    }
    info.tex_grids = 0;
    info.allocated = false;

    // Update the buffer in device memory.
    d_precomputed_info.copy_from(h_precomputed_info);
}

void Precomputed_prim::update_info(int _id) {
    h_precomputed_info.resize(max((int) h_precomputed_info.size(), _id+1));
    d_precomputed_info.realloc(h_precomputed_info.size());
    dp_precomputed_info = d_precomputed_info.ptr();

    PrecomputedInfo &info = h_precomputed_info[_id];
    info.id = _id;

    dealloc(_id);

    // XXX this is still global, not per prim
    if(Precomputed_env::d_anim_transform.ptr() != NULL)
    {
        cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeLinear;
        resDesc.res.linear.devPtr = Precomputed_env::d_anim_transform.ptr();
        resDesc.res.linear.desc = cudaCreateChannelDesc<float4>();
        resDesc.res.linear.sizeInBytes = Precomputed_env::d_anim_transform.size()*sizeof(Transfo);

        cudaTextureDesc tex;
        memset(&tex, 0, sizeof(tex));

        cudaCreateTextureObject(&info.tex_transform, &resDesc, &tex, NULL);
        CUDA_CHECK_ERRORS();
    }

    if(Precomputed_env::d_grad_transform.ptr() != NULL)
    {
        cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeLinear;
        resDesc.res.linear.devPtr = Precomputed_env::d_grad_transform.ptr();
        resDesc.res.linear.desc = cudaCreateChannelDesc<float4>();
        resDesc.res.linear.sizeInBytes = Precomputed_env::d_grad_transform.size()*sizeof(Transfo);

        cudaTextureDesc tex;
        memset(&tex, 0, sizeof(tex));

        cudaCreateTextureObject(&info.tex_transform_grad, &resDesc, &tex, NULL);
        CUDA_CHECK_ERRORS();
    }

    if(Precomputed_env::d_offset.ptr() != NULL)
    {
        cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeLinear;
        resDesc.res.linear.devPtr = Precomputed_env::d_offset.ptr();
        resDesc.res.linear.desc = cudaCreateChannelDesc<float4>();
        resDesc.res.linear.sizeInBytes = Precomputed_env::d_offset.size()*sizeof(int4);

        cudaTextureDesc tex;
        memset(&tex, 0, sizeof(tex));

        cudaCreateTextureObject(&info.tex_offset, &resDesc, &tex, NULL);
        CUDA_CHECK_ERRORS();
    }

    if(Precomputed_env::d_block.getCudaArray() != NULL)
    {
        cudaResourceDesc resDesc;
        memset(&resDesc, 0, sizeof(resDesc));
        resDesc.resType = cudaResourceTypeArray;
        resDesc.res.array.array = Precomputed_env::d_block.getCudaArray();

        cudaTextureDesc tex;
        memset(&tex, 0, sizeof(tex));
        tex.normalizedCoords = false;
        tex.filterMode = cudaFilterModeLinear;
        tex.addressMode[0] = cudaAddressModeClamp;
        tex.addressMode[1] = cudaAddressModeClamp;
        tex.addressMode[2] = cudaAddressModeClamp;

        cudaCreateTextureObject(&info.tex_grids, &resDesc, &tex, NULL);
        CUDA_CHECK_ERRORS();

        printf("created texture object %i\n", info.tex_grids);
    }
    else
    {
        printf("no grid texture object\n");
    }
    info.allocated = true;

    // Update the buffer in device memory.
    d_precomputed_info.copy_from(h_precomputed_info);
}

void Precomputed_prim::clear(){
    using namespace Precomputed_env;
    assert(_id >= 0);
    assert(_id < h_offset.size());
    assert(h_offset[_id].x >= 0);
    assert(nb_instances > 0);

    delete d_grids[_id];
    d_grids[_id] = NULL;

    // Deleted hrbf instances are tag with negative offsets in order to
    // re-use the element for a new instance
    h_offset[_id] = make_int4(-1, -1, -1, -1);
    d_offset.copy_from(h_offset);

    nb_instances--;

    Precomputed_prim::dealloc(_id);
}

__host__
void Precomputed_prim::fill_grid_with(Skeleton_env::Skel_id skel_id, const Bone* bone)
{
    using namespace Precomputed_env;

    assert(_id < h_offset.size());
    assert(_id >= 0);
    assert(h_offset[_id].x >= 0); // means the instance was deleted
    assert(nb_instances > 0);

    Bone::Id bone_id = bone->get_bone_id();
    OBBox_cu obbox = bone->get_obbox();

    // Compute the primive's grid
    fill_grid(bone_id, skel_id, obbox, GRID_RES, (*d_grids[_id]));

    // Adding the transformation to evaluate the grid
    Transfo t = world_coord_to_grid(obbox, GRID_RES);
    d_anim_transform.set(_id, t);
    d_grad_transform.set(_id, Transfo::identity());
    h_grid_transform[_id] = t;
    h_user_transform[_id] = Transfo::identity();

    // OK maybe its a bit slow to do it each time.
    // The best would be to do it once when all grids are loadeds
    copy_grids_to_cuarray_block();

    Precomputed_env::update_all_info();
}

__device__
float Precomputed_prim::f(const Point_cu& x) const
{
    using namespace Precomputed_env;

    const PrecomputedInfo &info = dp_precomputed_info[_id];
    return fetch_potential(info, x);
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu Precomputed_prim::gf(const Point_cu& x) const
{
    using namespace Precomputed_env;

    const PrecomputedInfo &info = dp_precomputed_info[_id];
    return fetch_gradient(info, x);
}

// -----------------------------------------------------------------------------

__device__
float Precomputed_prim::fngf(Vec3_cu& grad, const Point_cu& p) const
{
    using namespace Precomputed_env;

    const PrecomputedInfo &info = dp_precomputed_info[_id];
    Point_cu  r = fetch_transform(info) * p;
    Vec3_cu off = fetch_offset(info);

    r = r + off;
    if( is_in_grid( r, off ) )
    {
        float pot = fetch_grid(info, r, grad);
        grad = fetch_transform_grad(info) * grad;
        return pot;
    }

    grad = Vec3_cu(0.f, 0.f, 0.f);
    return 0.f;
}

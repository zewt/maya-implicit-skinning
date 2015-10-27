//#include "glsave.hpp"
//#include "blending_env_tex.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


#define TL1D(tex,x,dimx) (tex1D((tex),((dimx)-1)*(x)+0.5f))
#define TL2D(tex,x,y,dimx,dimy) (tex2D((tex),((dimx)-1)*(x)+0.5f, ((dimy)-1)*(y)+0.5f))
#define TL3D(tex,x,y,z,dimx,dimy,dimz) (tex3D((tex),((dimx)-1)*(x)+0.5f, ((dimy)-1)*(y)+0.5f,((dimz)-1)*(z)+0.5f))

// =============================================================================
namespace Blending_env {
// =============================================================================

// boundary functions fetch ------------------------------------------------
__device__
static float pan_hyperbola_fetch(float t){
    return TL1D(opening_hyperbola_tex,t*0.5f,NB_SAMPLES);
}
// -----------------------------------------------------------------------------

// profile functions fetch -------------------------------------------------
__device__ static float
hyperbola_fetch(float tan_t){
    return TL1D(profile_hyperbola_tex,tan_t,NB_SAMPLES);
}

__device__ static float2
hyperbola_normal_fetch(float tan_t){
    return TL1D(profile_hyperbola_normals_tex,tan_t,NB_SAMPLES);
}

__device__ static float
skin_fetch(float tan_t){
    return TL1D(profile_bulge_tex,tan_t,NB_SAMPLES);
}

__device__ static float2
skin_normal_fetch(float tan_t){
    return TL1D(profile_bulge_normals_tex,tan_t,NB_SAMPLES);
}

__device__
static float profile_bulge_4D_fetch(float tan_t, float strength)
{
    int idx = floorf(strength * (NB_SAMPLES_MAG_4D_BULGE-1));
    float x = idx*(NB_SAMPLES+2) + tan_t*(NB_SAMPLES-1);
    return tex1D(profiles_bulge_4D_tex, x + 0.5f);
}

__device__
static float2 profile_bulge_4D_normal_fetch(float tan_t, float strength)
{
    int idx = floorf(strength * (NB_SAMPLES_MAG_4D_BULGE-1));
    float x = idx*(NB_SAMPLES+2) + tan_t*(NB_SAMPLES-1);
    return tex1D(profiles_bulge_4D_normals_tex, x + 0.5f);
}

__device__
static float profile_ricci_4D_fetch(float tan_t, float N)
{
    int idx = floorf( N  * 2);
    float x = idx*(NB_SAMPLES+2) + tan_t*(NB_SAMPLES-1);
    return tex1D(profiles_ricci_4D_tex, x + 0.5f);
}
__device__
static float2 profile_ricci_4D_normal_fetch(float tan_t, float N)
{
    int idx = floorf( N * 2 );
    float x = idx*(NB_SAMPLES+2) + tan_t*(NB_SAMPLES-1);
    return tex1D(profiles_ricci_4D_normals_tex, x + 0.5f);
}
// -----------------------------------------------------------------------------

// operator fetch for *_OH -----------------------------------------------------
// used for U_OH
__device__ static float
openable_clean_union_fetch(float f1, float f2, float tan_alpha){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::U_OH );
    if (id < 0)
        return 0.f;
    Idx3_cu idx = operator_idx_offset_fetch(id);
    return operator_fetch(idx, f1*2.f, f2*2.f, tan_alpha);
}

__device__ static float2
openable_clean_union_gradient_fetch(float f1, float f2, float tan_alpha){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::U_OH );
    if (id < 0)
        return make_float2(0.f, 0.f);
    Idx3_cu idx = operator_idx_offset_fetch(id);
    return operator_grad_fetch(idx, f1*2.f, f2*2.f, tan_alpha);
}
// used for B_OH
__device__ static float
openable_clean_skin_fetch(float f1, float f2, float tan_alpha){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::B_OH );
    if (id < 0)
        return 0.f;
    Idx3_cu idx = operator_idx_offset_fetch(id);
    return operator_fetch(idx, f1*2.f, f2*2.f, tan_alpha);
}

__device__ static float2
openable_clean_skin_gradient_fetch(float f1, float f2, float tan_alpha){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::B_OH );
    if (id < 0)
        return make_float2(0.f, 0.f);
    Idx3_cu idx = operator_idx_offset_fetch(id);
    return operator_grad_fetch(idx, f1*2.f, f2*2.f, tan_alpha);
}
// -----------------------------------------------------------------------------

// 4D operators stuff ----------------------------------------------------------
// TODO: 4D fetch that fetch both potential and gradient.
// B_OH_4D
__device__
static float magnitude_3D_bulge_fetch(){
    return tex1Dfetch(magnitude_3D_bulge_tex, 0);
}

__device__ static float
openable_bulge_4D_fetch(float f1, float f2, float tan_alpha, float strength)
{
    int idx = floorf(strength * (NB_SAMPLES_MAG_4D_BULGE-1));

    int3 block_idx = {   (idx) % BLOCK_4D_BULGE_LX,
                        ((idx) / BLOCK_4D_BULGE_LX) % BLOCK_4D_BULGE_LZ,
                         (idx) / (BLOCK_4D_BULGE_LX * BLOCK_4D_BULGE_LY)
                     };

    block_idx.x = GRIG_4D_BULGE_LX * block_idx.x + 1/*padding*/;
    block_idx.y = GRIG_4D_BULGE_LY * block_idx.y + 1/*padding*/;
    block_idx.z = GRIG_4D_BULGE_LZ * block_idx.z + 1/*padding*/;

    float3 coords = { block_idx.x + f1*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.y + f2*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.z + tan_alpha * (NB_SAMPLES_4D_BULGE-1)
                    };

    return tex3D(openable_bulge_4D_tex, coords.x + 0.5f, coords.y + 0.5f, coords.z + 0.5f) * 0.5f;
}

__device__ static float2
openable_bulge_4D_gradient_fetch(float f1, float f2, float tan_alpha, float strength)
{
    int idx = floorf(strength * (NB_SAMPLES_MAG_4D_BULGE-1));

    int3 block_idx = {   (idx) % BLOCK_4D_BULGE_LX,
                        ((idx) / BLOCK_4D_BULGE_LX) % BLOCK_4D_BULGE_LZ,
                         (idx) / (BLOCK_4D_BULGE_LX * BLOCK_4D_BULGE_LY)
                     };

    block_idx.x = GRIG_4D_BULGE_LX * block_idx.x + 1/*padding*/;
    block_idx.y = GRIG_4D_BULGE_LY * block_idx.y + 1/*padding*/;
    block_idx.z = GRIG_4D_BULGE_LZ * block_idx.z + 1/*padding*/;

    float3 coords = { block_idx.x + f1*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.y + f2*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.z + tan_alpha * (NB_SAMPLES_4D_BULGE-1)
                    };

    return tex3D(openable_bulge_4D_gradient_tex, coords.x + 0.5f, coords.y + 0.5f, coords.z + 0.5f);
}

// R_OH_4D

__device__
static float n_3D_ricci_fetch(){
    return tex1Dfetch(n_3D_ricci_tex, 0);
}

__device__ static float
openable_ricci_4D_fetch(float f1, float f2, float tan_alpha, float N)
{
    int idx = floorf( N * 2 );

    int3 block_idx = {   (idx) % BLOCK_4D_BULGE_LX,
                        ((idx) / BLOCK_4D_BULGE_LX) % BLOCK_4D_BULGE_LZ,
                         (idx) / (BLOCK_4D_BULGE_LX * BLOCK_4D_BULGE_LY)
                     };

    block_idx.x = GRIG_4D_BULGE_LX * block_idx.x + 1/*padding*/;
    block_idx.y = GRIG_4D_BULGE_LY * block_idx.y + 1/*padding*/;
    block_idx.z = GRIG_4D_BULGE_LZ * block_idx.z + 1/*padding*/;

    float3 coords = { block_idx.x + f1*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.y + f2*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.z + tan_alpha * (NB_SAMPLES_4D_BULGE-1)
                    };

    return tex3D(openable_ricci_4D_tex, coords.x + 0.5f, coords.y + 0.5f, coords.z + 0.5f) * 0.5f;
}
__device__ static float2
openable_ricci_4D_gradient_fetch(float f1, float f2, float tan_alpha, float N)
{
    int idx = floorf( N * 2);

    int3 block_idx = {   (idx) % BLOCK_4D_BULGE_LX,
                        ((idx) / BLOCK_4D_BULGE_LX) % BLOCK_4D_BULGE_LZ,
                         (idx) / (BLOCK_4D_BULGE_LX * BLOCK_4D_BULGE_LY)
                     };

    block_idx.x = GRIG_4D_BULGE_LX * block_idx.x + 1/*padding*/;
    block_idx.y = GRIG_4D_BULGE_LY * block_idx.y + 1/*padding*/;
    block_idx.z = GRIG_4D_BULGE_LZ * block_idx.z + 1/*padding*/;

    float3 coords = { block_idx.x + f1*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.y + f2*2.f    * (NB_SAMPLES_4D_BULGE-1),
                      block_idx.z + tan_alpha * (NB_SAMPLES_4D_BULGE-1)
                    };

    return tex3D(openable_ricci_4D_gradient_tex, coords.x + 0.5f, coords.y + 0.5f, coords.z + 0.5f);
}
// -----------------------------------------------------------------------------



__device__ static float2
global_controller_fetch(float dot){
    return tex1D(global_controller_tex, dot * 0.5f + 0.5f);
}

__device__
static float2 controller_fetch(int inst_id, float dot){

    int2 bidx_2D = {inst_id  %  BLOCK_CTRL_LX, inst_id   / BLOCK_CTRL_LX  };
    int2 gidx_2D = {bidx_2D.x * GRID_CTRL_LX , bidx_2D.y * GRID_CTRL_LY   };

    const float off = 1/*padding*/ + 0.5f/*linear interpolation*/ + (dot * 0.5f + 0.5f) * (NB_SAMPLES-1);

    return tex2D(tex_controllers,
                 (float)gidx_2D.x + off,
                 (float)gidx_2D.y + 1.f/*middle column*/ + 0.5f /*linear interpolation*/);
}

// =============================================================================

// =============================================================================
// ======================  TEST with new env archi  ============================
// =============================================================================
__device__ static Idx3_cu operator_idx_offset_fetch(Op_id op_id){

    const int4 i = tex1Dfetch( tex_pred_operators_idx_offsets, op_id );
    return Idx3_cu(Vec3i_cu(i.x, i.y, i.z), i.w);
}

__device__ static float
operator_fetch(Idx3_cu tex_idx, float f1, float f2 ,float tan_alpha){
    int a, b, c;
    tex_idx.to_3d(a, b, c);
    return tex3D(tex_operators_values, a+f1*(NB_SAMPLES_OCU-1)+0.5f,
                                       b+f2*(NB_SAMPLES_OCU-1)+0.5f,
                                       c+tan_alpha*(NB_SAMPLES_ALPHA-1)+0.5f)*0.5f;
}

__device__ static float2
operator_grad_fetch(Idx3_cu tex_idx, float f1, float f2, float tan_alpha){
    int a, b, c;
    tex_idx.to_3d(a, b, c);
    return tex3D(tex_operators_grads, a+f1*(NB_SAMPLES_OCU-1)+0.5f,
                                      b+f2*(NB_SAMPLES_OCU-1)+0.5f,
                                      c+tan_alpha*(NB_SAMPLES_ALPHA-1)+0.5f);
}

__device__ static Op_id
predefined_op_id_fetch(Op_t op_t)
{
    // TODO: assert if wrong type of operators
    int id_opt = op_t - BINARY_3D_OPERATOR_BEGIN - 1;
    return tex1Dfetch(tex_pred_operators_id, id_opt);
}
// -----------------------------------------------------------------------------


}// END PRECOMPUTED_FUNCTIONS NAMESPACE ========================================

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
//#include "glsave.hpp"
//#include "globals.hpp"
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

// -----------------------------------------------------------------------------

static void bind_local()
{
    if(allocated){

        // Openings ---------------
        opening_hyperbola_tex.normalized = false;
        opening_hyperbola_tex.addressMode[0] = cudaAddressModeClamp;
        opening_hyperbola_tex.addressMode[1] = cudaAddressModeClamp;
        opening_hyperbola_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(opening_hyperbola_tex, d_pan_hyperbola));
        // END Openings -----------

        // Profiles ---------------
        profile_hyperbola_tex.normalized = false;
        profile_hyperbola_tex.addressMode[0] = cudaAddressModeClamp;
        profile_hyperbola_tex.addressMode[1] = cudaAddressModeClamp;
        profile_hyperbola_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_hyperbola_tex, d_hyperbola_profile));

        profile_hyperbola_normals_tex.normalized = false;
        profile_hyperbola_normals_tex.addressMode[0] = cudaAddressModeClamp;
        profile_hyperbola_normals_tex.addressMode[1] = cudaAddressModeClamp;
        profile_hyperbola_normals_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_hyperbola_normals_tex, d_hyperbola_normals_profile));

        profile_bulge_tex.normalized = false;
        profile_bulge_tex.addressMode[0] = cudaAddressModeClamp;
        profile_bulge_tex.addressMode[1] = cudaAddressModeClamp;
        profile_bulge_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_bulge_tex, d_bulge_profile));

        profile_bulge_normals_tex.normalized = false;
        profile_bulge_normals_tex.addressMode[0] = cudaAddressModeClamp;
        profile_bulge_normals_tex.addressMode[1] = cudaAddressModeClamp;
        profile_bulge_normals_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profile_bulge_normals_tex, d_bulge_profile_normals));

        // 4D BULGE --------------
        if(d_magnitude_3D_bulge)
        {
            magnitude_3D_bulge_tex.normalized = false;
            magnitude_3D_bulge_tex.addressMode[0] = cudaAddressModeWrap;
            magnitude_3D_bulge_tex.addressMode[1] = cudaAddressModeWrap;
            magnitude_3D_bulge_tex.filterMode = cudaFilterModePoint;
            CUDA_SAFE_CALL(cudaBindTexture(0, magnitude_3D_bulge_tex, d_magnitude_3D_bulge, sizeof(float)));
        }

        if(d_bulge_4D_profiles)
        {
            profiles_bulge_4D_tex.normalized = false;
            profiles_bulge_4D_tex.addressMode[0] = cudaAddressModeClamp;
            profiles_bulge_4D_tex.addressMode[1] = cudaAddressModeClamp;
            profiles_bulge_4D_tex.filterMode = cudaFilterModeLinear;
            CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_bulge_4D_tex, d_bulge_4D_profiles));
        }

        if(d_bulge_4D_profiles_normals)
        {
            profiles_bulge_4D_normals_tex.normalized = false;
            profiles_bulge_4D_normals_tex.addressMode[0] = cudaAddressModeClamp;
            profiles_bulge_4D_normals_tex.addressMode[1] = cudaAddressModeClamp;
            profiles_bulge_4D_normals_tex.filterMode = cudaFilterModeLinear;
            CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_bulge_4D_normals_tex, d_bulge_4D_profiles_normals));
        }

        openable_bulge_4D_tex.normalized = false;
        openable_bulge_4D_tex.addressMode[0] = cudaAddressModeClamp;
        openable_bulge_4D_tex.addressMode[1] = cudaAddressModeClamp;
        openable_bulge_4D_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_bulge.bind_tex(openable_bulge_4D_tex);

        openable_bulge_4D_gradient_tex.normalized = false;
        openable_bulge_4D_gradient_tex.addressMode[0] = cudaAddressModeClamp;
        openable_bulge_4D_gradient_tex.addressMode[1] = cudaAddressModeClamp;
        openable_bulge_4D_gradient_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_bulge_gradient.bind_tex(openable_bulge_4D_gradient_tex);
        // END 4D BULGE --------------

        // 4D RICCI --------------
        if(d_n_3D_ricci)
        {
            n_3D_ricci_tex.normalized = false;
            n_3D_ricci_tex.addressMode[0] = cudaAddressModeWrap;
            n_3D_ricci_tex.addressMode[1] = cudaAddressModeWrap;
            n_3D_ricci_tex.filterMode = cudaFilterModePoint;
            CUDA_SAFE_CALL(cudaBindTexture(0, n_3D_ricci_tex, d_n_3D_ricci, sizeof(float)));
        }

        profiles_ricci_4D_tex.normalized = false;
        profiles_ricci_4D_tex.addressMode[0] = cudaAddressModeClamp;
        profiles_ricci_4D_tex.addressMode[1] = cudaAddressModeClamp;
        profiles_ricci_4D_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_ricci_4D_tex, d_ricci_4D_profiles));

        profiles_ricci_4D_normals_tex.normalized = false;
        profiles_ricci_4D_normals_tex.addressMode[0] = cudaAddressModeClamp;
        profiles_ricci_4D_normals_tex.addressMode[1] = cudaAddressModeClamp;
        profiles_ricci_4D_normals_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(profiles_ricci_4D_normals_tex, d_ricci_4D_profiles_normals));

        openable_ricci_4D_tex.normalized = false;
        openable_ricci_4D_tex.addressMode[0] = cudaAddressModeClamp;
        openable_ricci_4D_tex.addressMode[1] = cudaAddressModeClamp;
        openable_ricci_4D_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_ricci.bind_tex(openable_ricci_4D_tex);

        openable_ricci_4D_gradient_tex.normalized = false;
        openable_ricci_4D_gradient_tex.addressMode[0] = cudaAddressModeClamp;
        openable_ricci_4D_gradient_tex.addressMode[1] = cudaAddressModeClamp;
        openable_ricci_4D_gradient_tex.filterMode = cudaFilterModeLinear;
        d_block_3D_ricci_gradient.bind_tex(openable_ricci_4D_gradient_tex);
        // END 4D RICCI --------------
        // END Profiles --------------

        // Controllers ---------------
        global_controller_tex.normalized = true;
        global_controller_tex.addressMode[0] = cudaAddressModeClamp;
        global_controller_tex.addressMode[1] = cudaAddressModeClamp;
        global_controller_tex.filterMode = cudaFilterModeLinear;
        CUDA_SAFE_CALL(cudaBindTextureToArray(global_controller_tex, d_global_controller));

        if( d_controllers != 0)
        {
            tex_controllers.normalized = false;
            tex_controllers.addressMode[0] = cudaAddressModeClamp;
            tex_controllers.addressMode[1] = cudaAddressModeClamp;
            tex_controllers.filterMode = cudaFilterModeLinear;
            CUDA_SAFE_CALL(cudaBindTextureToArray(tex_controllers, d_controllers));
        }
        // End Controllers -----------


        // Binary 3D operators -------
        d_operators_idx_offsets.bind_tex(tex_pred_operators_idx_offsets);
        d_operators_id.bind_tex(tex_pred_operators_id);

        tex_operators_values.normalized = false;
        tex_operators_values.addressMode[0] = cudaAddressModeClamp;
        tex_operators_values.addressMode[1] = cudaAddressModeClamp;
        tex_operators_values.filterMode = cudaFilterModeLinear;
        if (d_operators_values)
            CUDA_SAFE_CALL(cudaBindTextureToArray(tex_operators_values, d_operators_values));

        tex_operators_grads.normalized = false;
        tex_operators_grads.addressMode[0] = cudaAddressModeClamp;
        tex_operators_grads.addressMode[1] = cudaAddressModeClamp;
        tex_operators_grads.filterMode = cudaFilterModeLinear;
        if (d_operators_grads)
            CUDA_SAFE_CALL(cudaBindTextureToArray(tex_operators_grads, d_operators_grads));
        // End Binary 3D operators ---
    }
}

// -----------------------------------------------------------------------------

static void unbind_local()
{
    if(!allocated)
        return;
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_hyperbola_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(opening_hyperbola_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_hyperbola_normals_tex)     );
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_bulge_tex)                 );
    CUDA_SAFE_CALL( cudaUnbindTexture(profile_bulge_normals_tex)         );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_bulge_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_bulge_4D_normals_tex)     );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_ricci_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(profiles_ricci_4D_normals_tex)     );
    CUDA_SAFE_CALL( cudaUnbindTexture(magnitude_3D_bulge_tex)            );
    CUDA_SAFE_CALL( cudaUnbindTexture(n_3D_ricci_tex)                    );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_bulge_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_bulge_4D_gradient_tex)    );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_ricci_4D_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(openable_ricci_4D_gradient_tex)    );
    CUDA_SAFE_CALL( cudaUnbindTexture(global_controller_tex)             );
    CUDA_SAFE_CALL( cudaUnbindTexture(tex_controllers)                   );

    // =========================================================================
    // ======================  TEST with new env archi  ========================
    // =========================================================================
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_pred_operators_idx_offsets));
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_pred_operators_id));
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_operators_values));
    CUDA_SAFE_CALL(cudaUnbindTexture(tex_operators_grads));
    // -----------------------------------------------------------------------------
}

// -----------------------------------------------------------------------------

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

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
#ifndef BLENDING_ENV_HPP__
#define BLENDING_ENV_HPP__

/** @namespace Blending_env
    @brief Environement for blending operators stored into texture

    This namespace exports the functions that precompute the blending operator
    and the controller into CUDA textures. Computation is done on CPU so it is
    for now quiet slow. This is why we store operators into the hardrive. but
    in case you modify the operators settings you must recall to erase the cache
    holding the operators

    While the storage is here computation of blending operators is done through
    the namespace IBL.

    @see IBL
 */

#include <cstdio>
#include "cuda_utils.hpp"
#include "idx3_cu.hpp"

// forward defs ----------------------------------------------------------------
#include "blending_env_type.hpp"

namespace IBL {
    class Ctrl_setup;
    namespace Profile_polar {
        class Base;
    }
    namespace Opening {
        class Base;
    }
}
// END forward defs ------------------------------------------------------------

// =============================================================================
namespace Blending_env {
// =============================================================================

#ifndef MAX_TEX_LENGTH
#define MAX_TEX_LENGTH 2048 ///< For compute capability 1.0 2.x
#endif

#define MAX_2D_TEX_LENGTH_X 65536 ///< For compute capability 1.x
#define MAX_2D_TEX_LENGTH_Y 32768 ///< For compute capability 1.x

// -------------------

/// Number of samples for 1D functions such as controller, profile and opening
#define NB_SAMPLES (1000)

// -------------------

/// Number of samples in the xy plane of the 3D blending operators.
#define NB_SAMPLES_OCU (128)
/// Number of samples for the aperture of the 3D blending operators.
#define NB_SAMPLES_ALPHA (128)

// -------------------

/// Number of samples for the strength of the 4D bulge in contact
#define NB_SAMPLES_MAG_4D_BULGE (5)
/// Number of samples for the x,y,alpha direction of the 4D bulge
#define NB_SAMPLES_4D_BULGE (64)

/// Number of elements in a grid containing a 3D bulge plus padding
#define GRIG_4D_BULGE_LX (NB_SAMPLES_4D_BULGE + 2/*padding*/)
#define GRIG_4D_BULGE_LY (NB_SAMPLES_4D_BULGE + 2/*padding*/)
#define GRIG_4D_BULGE_LZ (NB_SAMPLES_4D_BULGE + 2/*padding*/)

/// Number storable grids in the (x, y, z) direction for a block
/// (with padding)
#define BLOCK_4D_BULGE_LX (MAX_TEX_LENGTH / (GRIG_4D_BULGE_LX))
#define BLOCK_4D_BULGE_LY (MAX_TEX_LENGTH / (GRIG_4D_BULGE_LY))
#define BLOCK_4D_BULGE_LZ (MAX_TEX_LENGTH / (GRIG_4D_BULGE_LZ))

// -------------------

#define SIZE_CONTROLLER (NB_SAMPLES + 2 /*padding*/)

/// Size of the controller's grid with padding
#define GRID_CTRL_LX (SIZE_CONTROLLER)
#define GRID_CTRL_LY 3

/// Number of storable controller's grids in the (x, y) direction
#define BLOCK_CTRL_LX (MAX_2D_TEX_LENGTH_X / (GRID_CTRL_LX))
#define BLOCK_CTRL_LY (MAX_2D_TEX_LENGTH_Y / (GRID_CTRL_LY))

// -----------------------------------------------------------------------------
extern std::string g_cache_dir;

extern cudaArray* d_bulge_profile;
extern cudaArray* d_bulge_profile_normals;

extern cudaArray* d_bulge_4D_profiles;
extern cudaArray* d_bulge_4D_profiles_normals;

extern cudaArray* d_ricci_4D_profiles;
extern cudaArray* d_ricci_4D_profiles_normals;
extern float* d_n_3D_ricci;
extern float  h_n_3D_ricci;

extern float* d_magnitude_3D_bulge;
extern float  h_magnitude_3D_bulge;

extern Cuda_utils::Device::CuArray<float>  d_block_3D_bulge;
extern Cuda_utils::Device::CuArray<float2>  d_block_3D_bulge_gradient;

extern Cuda_utils::Device::CuArray<float>  d_block_3D_ricci;
extern Cuda_utils::Device::CuArray<float2>  d_block_3D_ricci_gradient;

/// 2D array which actually stores a 1D function -> the controller
/// @note we use 2D instead of a 1D array because 1D array are limited to 655356
/// elements.
extern cudaArray* d_controllers;

extern cudaArray* d_hyperbola_profile;
extern cudaArray* d_hyperbola_normals_profile;

extern float*  h_hyperbola_profile;
extern float2* h_hyperbola_normals_profile;

extern float2* h_bulge_normals_profile;

extern cudaArray* d_global_controller;
extern IBL::Ctrl_setup globale_ctrl_shape;
extern const int nb_samples;

extern bool allocated;

extern cudaArray* d_pan_hyperbola;

// -----------------------------------------------------------------------------

/// Compute the bulge in contact with the magnitude set by
/// 'set_bulge_magnitude()'
/// @warning it is very slow !
void update_3D_bulge();

// -----------------------------------------------------------------------------
/// @name Controllers instance management
// -----------------------------------------------------------------------------

int get_nb_ctrl_instances();

Ctrl_id new_ctrl_instance();

void delete_ctrl_instance(Ctrl_id inst_id);

void update_controller(Ctrl_id inst_id, const IBL::Ctrl_setup& shape);

IBL::Ctrl_setup get_global_ctrl_shape();

float eval_global_ctrl(float dot);

void set_global_ctrl_shape(const IBL::Ctrl_setup& shape);
void set_bulge_magnitude(float mag);
void set_ricci_n(float N);

// -----------------------------------------------------------------------------

extern bool binded;
void unbind();
void bind();



// =============================================================================
// ======================  TEST with new env archi  ============================
// ============================== only ops =====================================
// USE :
//      // to load global operators from cache textures
//      init_env();
//      // to add custom operator with custom profile and opening
//      int new_c_op_id_1 = new_op_instance(profile, opening);
//      // to save custom operator into cache texture "filename"
//      make_cache(new_c_op_id_1, filename);
//      // to add custom operator from cache texture "filename"
//      int new_c_op_id_2 = new_op_instance(filename);
//      // to delete custom operator
//      delete_op_instance(new_c_op_id_1);
//      // to register/unregister newly added/deleted operators
//      update_operators();
//      // to save active env in "filename" file
//      make_cache_env(filename);
//      // to load active env from "filename" file
//      init_env_from_cache(filename);
//      // to free memory
//      clean_env();
//
// TODO : for other env stuff
// =============================================================================

extern cudaArray* d_operators_values;
extern cudaArray* d_operators_grads;
extern Cuda_utils::Device::Array<int4> d_operators_idx_offsets;
extern Cuda_utils::Device::Array<Op_id> d_operators_id;

/// Generates predefined profiles and openings, global controller,
/// enabled predefined operators and activates custom operators
void init_env();

/// upload operators to gpu memory
void update_operators();

/// clean all operators (enabled predefined and custom)
void clean_env();

// -----------------------------------------------------------------------------
/// @name Custom operators (User defined)
// -----------------------------------------------------------------------------

/// Adds a custom binary 3D operator to Blending_env.
/// The custom operator will be generated with 'profile' and 'opening'
/// @returns The new operator's identifier in Blending_env
/// @warning This is effective only after calling update_operators()
/// @see Blending_env::update_operators()
Op_id new_op_instance(const IBL::Profile_polar::Base& profile,
                      const IBL::Opening::Base& opening);

/// Adds a custom binary 3D operator to Blending_env.
/// The custom operator will be loaded from the cache texture specified by
/// 'filename'
/// @returns The new operator's identifier in Blending_env
/// @warning This is effective only after calling
/// @see Blending_env::update_operators()
Op_id new_op_instance(const std::string &filename);

/// remove the custom operator with @param op_id identifier from Blending_env
/// @warning This is effective only after calling
/// @see Blending_env::update_operators()
// what if call on deleted operator? => for now, idem as call on pred[0]
void delete_op_instante(Op_id op_id);

/// stores the specified custom operator into the cache texture specified by
/// @param filename
void make_cache(Op_id op_id, const std::string &filename);

// -----------------------------------------------------------------------------
/// @name predefined operators handling
/// predefined blending operators can be configured before init_env() is called.
// -----------------------------------------------------------------------------

/// Enables or disables the specified predefined operator according to 'on'
/// @warning Acts only when init_env() or reset_env() are called
void enable_predefined_operator( Op_t op_t, bool on );

/// @return number of predefined operators enabled through
/// enable_predefined_operator()
int get_nb_predefined_enabled();

/// @returns the Op_id associated with the predefined operator specified or -1
/// If not enabled
Op_id get_predefined_op_id(Op_t op_t);

/// store Blending_env's operators (enabled predefined and custom) into the
/// cache texture specified by @param filename
void make_cache_env(const std::string &filename);

/// restore Blending_env's operators (enabled predefined and custom) from the
/// cache texture specified by @param filename
/// @returns True when loaded successfully, false otherwise. If load not
/// successfull, @see Blending_env is left cleaned.
bool init_env_from_cache(const std::string &filename);











    extern texture<float, 1, cudaReadModeElementType>  profile_hyperbola_tex;
    extern texture<float2, 1, cudaReadModeElementType> profile_hyperbola_normals_tex;

    extern texture<float, 1, cudaReadModeElementType>  profile_bulge_tex;
    extern texture<float2, 1, cudaReadModeElementType> profile_bulge_normals_tex;

    /// List of profile for the bulge of different strength
    //@{
    extern texture<float, 1, cudaReadModeElementType>  profiles_bulge_4D_tex;
    extern texture<float2, 1, cudaReadModeElementType> profiles_bulge_4D_normals_tex;
    //@}

    /// List of profile for the ricci of different N
    //@{
    extern texture<float, 1, cudaReadModeElementType>  profiles_ricci_4D_tex;
    extern texture<float2, 1, cudaReadModeElementType> profiles_ricci_4D_normals_tex;
    //@}

    extern texture<float, 1, cudaReadModeElementType>  opening_hyperbola_tex;

    // 4D stuff ----------------------------------------------------------------
    extern texture<float, 1, cudaReadModeElementType>  magnitude_3D_bulge_tex;
    extern texture<float, 3, cudaReadModeElementType>  openable_bulge_4D_tex;
    extern texture<float2, 3, cudaReadModeElementType> openable_bulge_4D_gradient_tex;

    extern texture<float, 1, cudaReadModeElementType>  n_3D_ricci_tex;
    extern texture<float, 3, cudaReadModeElementType>  openable_ricci_4D_tex;
    extern texture<float2, 3, cudaReadModeElementType> openable_ricci_4D_gradient_tex;
    // -------------------------------------------------------------------------

    extern texture<float2, 1, cudaReadModeElementType> global_controller_tex;

    extern texture<float2, 2, cudaReadModeElementType> tex_controllers;

    /// bind textures to the arrays into the '.cu' this header is included in
    static inline void bind_local();

    /// unbind textures to the arrays into the '.cu' this header is included in
    static inline void unbind_local();

    // boundary functions fetch ------------------------------------------------
    __device__
    static float pan_hyperbola_fetch(float t);
    // -------------------------------------------------------------------------

    // profile functions fetch -------------------------------------------------
    __device__
    static float hyperbola_fetch(float tan_t);

    __device__
    static float2 hyperbola_normal_fetch(float tan_t);

    __device__
    static float skin_fetch(float tan_t);

    __device__
    static float2 skin_normal_fetch(float tan_t);

    __device__
    static float profile_bulge_4D_fetch(float tan_t, float strength);

    __device__
    static float2 profile_bulge_4D_normal_fetch(float tan_t, float strength);

    __device__
    static float profile_ricci_4D_fetch(float tan_t, float N);

    __device__
    static float2 profile_ricci_4D_normal_fetch(float tan_t, float N);
    // -------------------------------------------------------------------------

    // operator fetch for OH ---------------------------------------------------
    // used for U_OH
    __device__
    static float openable_clean_union_fetch(float f1, float f2, float tan_alpha);

    __device__
    static float2 openable_clean_union_gradient_fetch(float f1, float f2, float tan_alpha);

    // used for B_OH
    __device__
    static float openable_clean_skin_fetch(float f1, float f2, float tan_alpha);

    __device__
    static float2 openable_clean_skin_gradient_fetch(float f1, float f2, float tan_alpha);
    // -------------------------------------------------------------------------

    // 4D operators stuff ------------------------------------------------------
    // TODO: 4D fetch that fetch both potential and gradient.
    // B_OH_4D
    __device__
    static float magnitude_3D_bulge_fetch();

    __device__
    static float openable_bulge_4D_fetch(float f1, float f2, float tan_alpha, float strength);

    __device__
    static float2 openable_bulge_4D_gradient_fetch(float f1, float f2, float tan_alpha, float strength);

    // R_OH_4D
    __device__
    static float n_3D_ricci_fetch();

    __device__
    static float openable_ricci_4D_fetch(float f1, float f2, float tan_alpha, float N);

    __device__
    static float2 openable_ricci_4D_gradient_fetch(float f1, float f2, float tan_alpha, float N);
    // -------------------------------------------------------------------------

    /// @param dot Is the angle between two gradient given by the dot product
    /// i.e cos(teta)
    __device__
    static float2 global_controller_fetch(float dot);

    /// @param dot Is the angle between two gradient given by the dot product
    /// i.e cos(teta)
    __device__
    static float2 controller_fetch(int inst_id, float dot);

    // =========================================================================
    // ======================  TEST with new env archi  ========================
    // =========================================================================
    // USE : Taking op_id as the operator-to-be-fetched's id
    //      // get operator's Idx3_cu for value and gradient fetch
    //      Idx3_cu idx = operator_idx_offset_fetch(op_id);
    //      // get operator's value when applied on f1 and f2 with an opening
    //      // value of tan_alpha
    //      float f = operator_fetch(idx, f1, f2, tan_alpha);
    //      // get operator's gradient when applied on f1 and f2 (whose
    //      // gradients are gf1 and gf2) with an opening value of tan_alpha
    //      float2 dg = operator_grad_fetch(idx, f1, f2, tan_alpha);
    //      Vec3_cu gf = dg*gf1 + dg*gf2;
    //
    // WARNING : keep 4d ops & profiles & openings & other env stuff
    // =========================================================================

    extern texture<int4  , 1, cudaReadModeElementType> tex_pred_operators_idx_offsets;
    extern texture<int   , 1, cudaReadModeElementType> tex_pred_operators_id;
    extern texture<float , 3, cudaReadModeElementType> tex_operators_values;
    extern texture<float2, 3, cudaReadModeElementType> tex_operators_grads;

    __device__
    static Idx3_cu operator_idx_offset_fetch(Op_id op_id);
    __device__
    static float operator_fetch(Idx3_cu tex_idx, float f1, float f2 ,float tan_alpha);
    __device__
    static float2 operator_grad_fetch(Idx3_cu tex_idx, float f1, float f2, float tan_alpha);

    /// @returns the identifier attached to the op_t predefined operator
    __device__
    static Op_id predefined_op_id_fetch( Op_t op_t );
    // -----------------------------------------------------------------------------


}// END BLENDING_ENV NAMESPACE =================================================

// Implementation of inline functions above:
#include "blending_env.inl"

#endif // BLENDING_ENV_HPP__

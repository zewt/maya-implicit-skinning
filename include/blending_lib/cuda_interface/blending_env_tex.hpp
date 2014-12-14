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
#ifndef BLENDING_ENV_TEX_HPP__
#define BLENDING_ENV_TEX_HPP__

/** This files handles the textures that store the precomputed values of
    the blending operators and controllers.

    <b>There is some restrictions with the use of cuda texture read carrefully
    what follows to use this header correctly </b>

    Cuda textures scope is limited to a single '.cu'. Inclusion of this header
    in several '.cu' will create DIFFERENT texture references inside
    each '.cu'. As a result in EACH '.cu' you MUST bind the textures before
    using them in a kernel. You can do it with 'bind_local()'. Don't forget to
    unbind with unbind_local().

    Sometimes you want to remotly bind/unbind textures inside a '.cu'.
    You can't use bind_local() for this you must use bind() in the header
    ****_env_tex_binding.hpp; This header can be included only once in the
    entire project. Moreover it must be included just after this header.
    Therefore remotly bind/unbind texture can be done only for a single .cu
    in the project.
 */

#include "blending_env.hpp"
#include "idx3_cu.hpp"
#include "cuda_utils.hpp"

// =============================================================================
namespace Blending_env{
// =============================================================================
    //texture storing precomputed inverse of the hyperbola
    texture<float, 1, cudaReadModeElementType>  profile_hyperbola_tex;
    texture<float2, 1, cudaReadModeElementType> profile_hyperbola_normals_tex;

    texture<float, 1, cudaReadModeElementType>  profile_bulge_tex;
    texture<float2, 1, cudaReadModeElementType> profile_bulge_normals_tex;

    /// List of profile for the bulge of different strength
    //@{
    texture<float, 1, cudaReadModeElementType>  profiles_bulge_4D_tex;
    texture<float2, 1, cudaReadModeElementType> profiles_bulge_4D_normals_tex;
    //@}

    /// List of profile for the ricci of different N
    //@{
    texture<float, 1, cudaReadModeElementType>  profiles_ricci_4D_tex;
    texture<float2, 1, cudaReadModeElementType> profiles_ricci_4D_normals_tex;
    //@}

    texture<float, 1, cudaReadModeElementType>  opening_hyperbola_tex;

    // 4D stuff ----------------------------------------------------------------
    texture<float, 1, cudaReadModeElementType>  magnitude_3D_bulge_tex;
    texture<float, 3, cudaReadModeElementType>  openable_bulge_4D_tex;
    texture<float2, 3, cudaReadModeElementType> openable_bulge_4D_gradient_tex;

    texture<float, 1, cudaReadModeElementType>  n_3D_ricci_tex;
    texture<float, 3, cudaReadModeElementType>  openable_ricci_4D_tex;
    texture<float2, 3, cudaReadModeElementType> openable_ricci_4D_gradient_tex;
    // -------------------------------------------------------------------------

    texture<float2, 1, cudaReadModeElementType> global_controller_tex;

    texture<float2, 2, cudaReadModeElementType> tex_controllers;

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

    texture<int4  , 1, cudaReadModeElementType> tex_pred_operators_idx_offsets;
    texture<int   , 1, cudaReadModeElementType> tex_pred_operators_id;
    texture<float , 3, cudaReadModeElementType> tex_operators_values;
    texture<float2, 3, cudaReadModeElementType> tex_operators_grads;

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

}// END Blending_env ===========================================================

#include "blending_env_tex.inl"

#endif //BLENDING_ENV_TEX_HPP__

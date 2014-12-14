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
#include "cuda_main_kernels.hpp"


/// @name Cuda textures importation
/// @{
#include "blending_env_tex.hpp"
#include "blending_env_tex_binding.hpp"

#include "hrbf_env_tex.hpp"
#include "hrbf_env_tex_binding.hpp"

#include "precomputed_prim_tex.hpp"
#include "precomputed_prim_env_binding.hpp"

#include "skeleton_env_tex.hpp"
#include "skeleton_env_tex_binding.hpp"

#include "textures_env_tex.hpp"

#include "constants_tex.hpp"
/// @}

/// @name Class implementation using the previous textures
/// @{
#include "hermiteRBF.inl"
#include "precomputed_prim.inl"
/// @}

/// @name Main cuda kernels
/// @{
#include "animesh_potential.hpp"
#include "raytracing.hpp"
#include "animesh_kers_colors.inl"
/// @}

#include "cuda_current_device.hpp"
#include "port_cuda_gl_interop.h"



#include "globals.hpp"


// -----------------------------------------------------------------------------
#include "timer.hpp"
void init_cuda(const std::vector<Blending_env::Op_t>& op)
{
    std::cout << "\n--- CUDA CONTEXT SETUP ---\n";
    // We choose the most efficient GPU and use it :
    int device_id = Cuda_utils::get_max_gflops_device_id();

    //CUDA_SAFE_CALL( cudaDeviceSynchronize() );
    //CUDA_SAFE_CALL( cudaDeviceReset() );

    // these two functions are said to be mutually exclusive
    //{
    /// MUST be called after OpenGL/Glew context are init and before any cuda calls that create context like malloc
    //CUDA_SAFE_CALL(cudaGLSetGLDevice(device_id) );
    CUDA_SAFE_CALL(cudaSetDevice(device_id) );
    //}

    //CUDA_SAFE_CALL( cudaDeviceSynchronize() );
    //Cuda_utils::print_device_attribs(get_cu_device() );


    // Compute on host implicit blending operators
    // and allocate them on device memory
    std::cout << "\nInitialize blending operators" << std::endl;

    std::cout << "GPU memory usage: \n";
    double free, total;
    Cuda_utils::get_device_memory_usage(free, total);
    std::cout << "free: " << free << " Mo\ntotal: " << total << " Mo" << std::endl;

    for(unsigned int i = 0; i < op.size(); ++i)
        Blending_env::enable_predefined_operator( op[i], true );

    Timer t; t.start();
    if (!Blending_env::init_env_from_cache("ENV_CACHE")){
        t.stop();
        Blending_env::init_env();
        Blending_env::make_cache_env("ENV_CACHE");
    }

    std::cout << "Operators loaded in: " << t.stop() << "s" << std::endl;

    Blending_env::bind();
    HRBF_env::bind();

    std::cout << "allocate float constants in device memory\n";
    Constants::allocate();
    std::cout << "Done\n";

    std::cout << "allocate textures for raytracing\n";
    Textures_env::load_envmap("resource/textures/env_maps/skymap.ppm");
    Textures_env::load_blob_tex("resource/textures/tex.ppm");
    Textures_env::load_extrusion_tex("resource/textures/test.ppm", 0);
    Textures_env::load_extrusion_tex("resource/textures/vortex_text.ppm", 1);
    Textures_env::init();
    std::cout << "Done\n";

    Skeleton_env::init_env();

    std::cout << "\n--- END CUDA CONTEXT SETUP ---" << std::endl;
}

// =============================================================================
namespace Raytracing {
// =============================================================================

static bool full_eval = false;

static Scene_Type scene_type = NONE;

// -----------------------------------------------------------------------------

/// Update the current bounding box to raytrace
void update_bbox(const BBox_cu& bbox){
    //CUDA_SAFE_CALL( cudaMemcpyToSymbol(bbox___, &bbox, sizeof(BBox_cu)) );/////////////////////////////////////
}

// -----------------------------------------------------------------------------

void set_partial_tree_eval(bool s){ full_eval = s; }

// -----------------------------------------------------------------------------

void set_bone_to_trace(const std::vector<int>& bone_ids)
{
    Skeleton_partial_eval::set_bones_to_raytrace(bone_ids);
}

// -----------------------------------------------------------------------------

#define RAYTRACE(TEMPLATE) raytrace_kernel<TEMPLATE><<<ctx.grid, ctx.block >>>( d_ctx.ptr() )

void trace( Context& ctx )
{

    Cuda_utils::Device::Array<Context> d_ctx(1);
    d_ctx.set(0, ctx);

    switch (scene_type) {
#if 0
#ifdef USE_MIN_OP
    case MIN_T:
        RAYTRACE(MinOperator);
        break;
#endif
#ifdef USE_MAX_OP
    case MAX_T:
        RAYTRACE(MaxOperator);
        break;
#endif
#ifdef USE_SUM_OP
    case SUM_T:
        RAYTRACE(SumOperator);
        break;
#endif
#ifdef USE_RICCI_OP
    case RICCI_T:
        RAYTRACE(RicciOperator);
        break;
#endif
#ifdef USE_SOFT_DIFF_OP
    case SOFT_DIFF_T:
        RAYTRACE(SoftDiffOperator);
        break;
#endif
#ifdef USE_SHARP_DIFF_OP
    case SHARP_DIFF_T:
        RAYTRACE(SharpDiffOperator);
        break;
#endif
#ifdef USE_CANI_OP
    case CANI_CONTACT_T:
        RAYTRACE(CaniContactOperator);
        break;
#endif
#ifdef USE_GRAPH_OP
    case Wyvill_GRAPH_OPERATOR_T:
        RAYTRACE(WyvillGraphOperator);
        break;
#endif
#ifdef USE_RESTRICTED_OP
    case RESTRICTED_BLENDING_T:
        RAYTRACE(RestrictedBlending);
        break;
#endif
#ifdef USE_CL_OP
    case CIRCLE_LINES:
        RAYTRACE(Circle_lines);
        break;
#endif
#ifdef USE_CHO_OP
    case CIRCLE_HYPERBOLA_OPEN:
        RAYTRACE(Circle_hyperbola_open);
        break;
#endif
#ifdef USE_CHC_OP
    case CIRCLE_HYPERBOLA_CLOSED_D:
        RAYTRACE(Circle_hyperbola_closed<-1>);
        break;
    case CIRCLE_HYPERBOLA_CLOSED_H:
        RAYTRACE(Circle_hyperbola_closed<0>);
        break;
    case CIRCLE_HYPERBOLA_CLOSED_T:
        RAYTRACE(Circle_hyperbola_closed<1>);
        break;
#endif
#ifdef USE_UHO_OP
    case ULTIMATE_HYPERBOLA_OPEN:
        RAYTRACE(Ultimate_open);
        break;
#endif
#ifdef USE_UHC_OP
    case ULTIMATE_HYPERBOLA_CLOSED_H:
        RAYTRACE(Ultimate_hyperbola_closed<0>);
        break;
    case ULTIMATE_HYPERBOLA_CLOSED_T:
        RAYTRACE(Ultimate_hyperbola_closed<1>);
        break;
#endif
#ifdef USE_BHO_OP
    case BULGE_HYPERBOLA_OPEN:
        RAYTRACE(Bulge_hyperbola_open);
        break;
#endif
#ifdef USE_BHC_OP
    case BULGE_HYPERBOLA_CLOSED_H:
        RAYTRACE(Bulge_hyperbola_closed<0>);
        break;
    case BULGE_HYPERBOLA_CLOSED_T:
        RAYTRACE(Bulge_hyperbola_closed<1>);
        break;
#endif
#ifdef USE_ICHO_OP
    case CIRCLE_INTERSECTION_HYPERBOLA_OPEN:
        RAYTRACE(ICircle_hyperbola_open);
        break;
#endif
#ifdef USE_ICHC_OP
    case CIRCLE_INTERSECTION_HYPERBOLA_CLOSED_H:
        RAYTRACE(ICircle_hyperbola_closed<0>);
        break;
    case CIRCLE_INTERSECTION_HYPERBOLA_CLOSED_T:
        RAYTRACE(ICircle_hyperbola_closed<1>);
        break;
#endif
#ifdef USE_IUHO_OP
    case ULTIMATE_INTERSECTION_HYPERBOLA_OPEN:
        RAYTRACE(IUltimate_hyperbola_open);
        break;
#endif
#ifdef USE_IUHC_OP
    case ULTIMATE_INTERSECTION_HYPERBOLA_CLOSED_H:
        RAYTRACE(IUltimate_hyperbola_closed<0>);
        break;
    case ULTIMATE_INTERSECTION_HYPERBOLA_CLOSED_T:
        RAYTRACE(IUltimate_hyperbola_closed<1>);
        break;
#endif
#ifdef USE_DCHO_OP
    case CIRCLE_DIFF_HYPERBOLA_OPEN:
        RAYTRACE(DCircle_hyperbola_open);
        break;
#endif
#ifdef USE_DCHC_OP
    case CIRCLE_DIFF_HYPERBOLA_CLOSED_H:
        RAYTRACE(DCircle_hyperbola_closed<0>);
        break;
    case CIRCLE_DIFF_HYPERBOLA_CLOSED_T:
        RAYTRACE(DCircle_hyperbola_closed<1>);
        break;
#endif
#ifdef USE_DUHO_OP
    case ULTIMATE_DIFF_HYPERBOLA_OPEN:
        RAYTRACE(DUltimate_hyperbola_open);
        break;
#endif
#ifdef USE_DUHC_OP
    case ULTIMATE_DIFF_HYPERBOLA_CLOSED_H:
        RAYTRACE(DUltimate_hyperbola_closed<0>);
        break;
    case ULTIMATE_DIFF_HYPERBOLA_CLOSED_T:
        RAYTRACE(DUltimate_hyperbola_closed<1>);
        break;
#endif
#endif
    case SKELETON:
#if 1
        if(full_eval)
        {
            RAYTRACE(Animesh_kers::Skeleton_potential);
        }
        else
        {
            RAYTRACE(Skeleton_partial_eval);
        }
#else
        for (int i = 0; i < 100; ++i) {
            std::cout << "HARD CODED ";
            std::cout << "RAYTRACING DISAAAAAAAAAAAAAAAAAAAAAAAAAABLE !!!!!!!!!!!!";
            std::cout << std::endl;
        }
        assert(false);
#endif
        break;
    case COMPOSITION_TREE:
#if 0
        if (g_blob_tree->isValid()) {
            int nb_threads = ctx.grid.x*ctx.block.x * ctx.grid.y*ctx.block.y * ctx.grid.z*ctx.block.z;
            g_blob_tree->to_gpu( nb_threads );
            RAYTRACE(CsgTreeEvaluator);
        }
#else
        for (int i = 0; i < 100; ++i) {
            std::cout << "HARD CODED ";
            std::cout << "RAYTRACING DISAAAAAAAAAAAAAAAAAAAAAAAAAABLE !!!!!!!!!!!!";
            std::cout << std::endl;
        }
        assert(false);
#endif
        break;

    default :
        break;
    }


    CUDA_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

void set_scene_type(Scene_Type type){
    scene_type = type;
}

void trace_skinning_skeleton(const Context& ctx) {

    Cuda_utils::Device::Array<Context> d_ctx(1);
    d_ctx.set(0, ctx);

#if 0
    RAYTRACE(Animesh_kers::Skeleton_potential);
#else
        for (int i = 0; i < 100; ++i) {
            std::cout << "HARD CODED ";
            std::cout << "RAYTRACING DISAAAAAAAAAAAAAAAAAAAAAAAAAABLE !!!!!!!!!!!!";
            std::cout << std::endl;
        }
        assert(false);
#endif
}


}// End Raytracing namespace ===================================================


__global__
void get_controller_values(Cuda_utils::DA_float2 out_vals, int inst_id)
{
    int n = out_vals.size();
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < n)
    {
        float dot = cosf(idx * (1.f / (float)n) * M_PI);
#if 1
        out_vals[idx] = Blending_env::controller_fetch(inst_id, dot);
#else
        out_vals[idx] = Blending_env::global_controller_fetch(dot);
#endif
    }
}

// -----------------------------------------------------------------------------


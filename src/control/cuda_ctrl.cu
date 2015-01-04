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
#include "cuda_ctrl.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "skeleton.hpp"
#include "globals.hpp"
#include "cuda_utils_common.hpp"
#include "constants.hpp"
#include "skeleton_env.hpp"
#include "blending_env.hpp"
#include "hrbf_env.hpp"
#include "cuda_current_device.hpp"
#include "constants_tex.hpp"
#include "timer.hpp"

namespace { __device__ void fix_debug() { } }

// =============================================================================
namespace Cuda_ctrl {
// =============================================================================

Debug_ctrl           _debug;
Operators_ctrl       _operators;

CudaCtrl::CudaCtrl()
{
    _anim_mesh = NULL;
    _mesh = NULL;
}

CudaCtrl::~CudaCtrl()
{
    delete _anim_mesh;
    delete _mesh;
}

void CudaCtrl::load_mesh( Mesh* mesh )
{
    delete _anim_mesh;
    _anim_mesh = 0;

    delete _mesh;
    _mesh = mesh;
    _mesh->check_integrity();

    std::cout << "mesh loaded" << std::endl;
}

bool CudaCtrl::is_animesh_loaded() const
{
    return _anim_mesh != 0;
}

bool CudaCtrl::is_skeleton_loaded() const { return _skeleton.is_loaded(); }

void CudaCtrl::load_animesh()
{
    delete _anim_mesh;
    _anim_mesh = new Animated_mesh_ctrl(_mesh, _skeleton.skel);
}

void get_mem_usage(double& total, double& free)
{
    Cuda_utils::get_device_memory_usage(free, total);
}

// -----------------------------------------------------------------------------

void init_host()
{
    std::cout << "Initialize constants\n";
    Constants::init();

    std::cout << "Done\n";
}

// -----------------------------------------------------------------------------

void set_default_controller_parameters()
{
#if 0
    //for bulge-free blending skinning (elbow)
    Constants::set(Constants::F0, 1.f);
    Constants::set(Constants::F1, 0.43f);
    Constants::set(Constants::F2, 1.f);
    Constants::set(Constants::B0, 0.2f);
    Constants::set(Constants::B1, 0.7f);
    Constants::set(Constants::B2, 1.2f);
    Constants::set(Constants::POW0, 1.f);
    Constants::set(Constants::POW1, 1.f);
#else
    Constants::set(Constants::F0, 0.5f );
    Constants::set(Constants::F1, 0.5f );
    Constants::set(Constants::F2, 0.5f );
    Constants::set(Constants::B0, 0.2f );
    Constants::set(Constants::B1, 0.7f );
    Constants::set(Constants::B2, 1.2f );
    Constants::set(Constants::POW0, 1.f);
    Constants::set(Constants::POW1, 1.f);
#endif
    //Blending_env::update_opening(); <- // TODO: to be deleted
    //Blending_env::set_global_ctrl_shape(shape);
}

// -----------------------------------------------------------------------------

void cuda_start(const std::vector<Blending_env::Op_t>& op)
{
    using namespace Cuda_ctrl;

#ifndef NDEBUG
    std::cout << "WARNING: you're still in debug mode" << std::endl;
#endif
    init_host();

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

    Skeleton_env::init_env();

    std::cout << "\n--- END CUDA CONTEXT SETUP ---" << std::endl;


    set_default_controller_parameters();

//    atexit(cleanup);
}

// -----------------------------------------------------------------------------

void cleanup()
{
    cudaDeviceSynchronize();
    CUDA_CHECK_ERRORS();

    Constants::free();

    Blending_env::clean_env();
    HRBF_env::clean_env();
    Skeleton_env::clean_env();

    CUDA_CHECK_ERRORS();

    cudaDeviceReset();
}

}// END CUDA_CTRL NAMESPACE  ===================================================

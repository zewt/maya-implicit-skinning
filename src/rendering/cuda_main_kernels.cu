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

namespace { __device__ void fix_debug() { } }

/// @name Cuda textures importation
/// @{
#include "blending_env.hpp"

#include "hrbf_env_tex.hpp"
#include "hrbf_env_tex_binding.hpp"

#include "skeleton_env.hpp"

#include "constants_tex.hpp"
/// @}

/// @name Class implementation using the previous textures
/// @{
#include "hermiteRBF.inl"
/// @}

/// @name Main cuda kernels
/// @{
#include "animesh_potential.hpp"
/// @}


#include "cuda_current_device.hpp"

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

    Skeleton_env::init_env();

    std::cout << "\n--- END CUDA CONTEXT SETUP ---" << std::endl;
}

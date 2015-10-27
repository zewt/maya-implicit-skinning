#include "cuda_ctrl.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
using namespace std;

#include "skeleton.hpp"
#include "cuda_utils_common.hpp"
#include "constants.hpp"
#include "skeleton_env.hpp"
#include "blending_env.hpp"
#include "hrbf_env.hpp"
#include "cuda_current_device.hpp"
#include "constants_tex.hpp"
#include "timer.hpp"

namespace Cuda_ctrl {

Debug_ctrl           _debug;
Operators_ctrl       _operators;

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
    // We choose the most efficient GPU and use it :
    int device_id = Cuda_utils::get_max_gflops_device_id();

    cudaError_t code = cudaSetDevice(device_id);
    if(code != cudaSuccess)
        throw std::runtime_error(cudaGetErrorString(code));

    cudaDeviceProp deviceProp;
    CUDA_SAFE_CALL(cudaGetDeviceProperties(&deviceProp, device_id));
    printf("Device %d: \"%s\"\n", device_id, deviceProp.name);
    printf("Compute Capability   : %d.%d\n", deviceProp.major, deviceProp.minor);

    Constants::init();

    //Cuda_utils::print_device_attribs(get_cu_device() );

    // Compute on host implicit blending operators and allocate them on device memory
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

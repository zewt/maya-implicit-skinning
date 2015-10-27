#ifndef CUDA_UTILS_COMMON_HPP__
#define CUDA_UTILS_COMMON_HPP__

#include <cassert>
#include <vector>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>
#include "cuda_compiler_interop.hpp"
#include "memory_debug.hpp"
#include "memory_debug.inl"

/** @namespace Cuda_utils::Common
    @brief Structures and functions shortcut to work on both device and host
    memory

    This file is part of the Cuda_utils homemade toolkit it provides common
    feature for device and host arrays

    @see Cuda_utils
*/

/// Check a cuda API call.
/// exit() if not cudaSuccess:
#define CUDA_SAFE_CALL(x) do{\
    cudaError_t code = x;\
    if(code != cudaSuccess){\
    fprintf(stderr,"CUDA error: %s at %s, line %d\n",\
    cudaGetErrorString(code), __FILE__, __LINE__);\
    fflush(stderr);\
    cuda_print_memory_trace();\
    cuda_print_rusage();\
    assert(false);\
    }\
    } while(0)

#define CU_SAFE_CALL(call) do{                                               \
    CUresult err = call;                                                     \
    if( CUDA_SUCCESS != err) {                                               \
        fprintf(stderr, "Cuda driver error %x in file '%s' in line %i.\n",   \
                err, __FILE__, __LINE__ );                                   \
        assert(false);                                                       \
    } }while(0)

/// Use this macro to check the latest cuda error. It exits when the test
/// failed. This is useful when you want to check kernels errors. You just
/// need to put the CUDA_CHECK_ERRORS(); right after the kernel call.
#define CUDA_CHECK_ERRORS() do { Cuda_utils::checkCudaErrors(__FILE__, __LINE__, false); } while(0)

// Like CUDA_CHECK_ERRORS, but check for errors regardless of the value of
// setCudaDebugChecking.
#define CUDA_CHECK_ERRORS_ALWAYS() do { Cuda_utils::checkCudaErrors(__FILE__, __LINE__, true); } while(0)

// =============================================================================
namespace Cuda_utils{
// =============================================================================

void checkCudaErrors(const char *file, int line, bool always);

// Enable or disable runtime CUDA error checking.  On by default in debug builds, off by
// default in release builds.  If disabled, errors are only checked when we'd need to
// stall the pipeline anyway, eg. at the end of the deformer.
void setCudaDebugChecking(bool value);
bool getCudaDebugChecking();

// =============================================================================
namespace Common{
// =============================================================================
/** @brief Mother class for Host and Device arrays of Cuda_utils namespace
    @see Cuda_utils Cuda_utils::Host::Array Cuda_utils::Device::Array
*/
template <class T>
struct Array{
    IF_CUDA_DEVICE_HOST
    inline int	size() const { return nb_elt; }

protected:
    IF_CUDA_DEVICE_HOST
    inline Array() : nb_elt(0) {}
    IF_CUDA_DEVICE_HOST
    inline Array(int i) : nb_elt(i) {}
    int nb_elt;
    enum{
        IS_ALLOCATED = 1,
        IS_COPY = 2        ///< If on the array won't be freed at destruction
    };
};

}
// END COMMON NAMESPACE ========================================================

/// Safe memory copy host to device
template <class T>
inline
void mem_cpy_htd(T* dst, const T* src, int nb_elt){
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(dst),
                              reinterpret_cast<const void*>(src),
                              nb_elt * sizeof(T),
                              cudaMemcpyHostToDevice) );
}

/// Safe memory copy device to host
template <class T>
inline
void mem_cpy_dth(T* dst, const T* src, int nb_elt){
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(dst),
                              reinterpret_cast<const void*>(src),
                              nb_elt * sizeof(T),
                              cudaMemcpyDeviceToHost) );
}

/// Safe memory copy device to device
template <class T>
inline
void mem_cpy_dtd(T* dst, const T* src, int nb_elt){
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(dst),
                              reinterpret_cast<const void*>(src),
                              nb_elt * sizeof(T),
                              cudaMemcpyDeviceToDevice) );
}

/// Safe memory copy host to host
template <class T>
inline
void mem_cpy_hth(T* dst, const T* src, int nb_elt){
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(dst),
                              reinterpret_cast<const void*>(src),
                              nb_elt * sizeof(T),
                              cudaMemcpyHostToHost) );
}

/// Safe memory copy from host to device constant memory
template <class T>
inline
void mem_cpy_symbol_htd(const char* symbol, const T* src, int nb_elt){
    CUDA_SAFE_CALL(cudaMemcpyToSymbol(symbol,
                                      reinterpret_cast<const void*>(src),
                                      nb_elt * sizeof(T),
                                      0,
                                      cudaMemcpyHostToDevice) );
}

/// Safe memory copy from device to device constant memory
template <class T>
inline
void mem_cpy_symbol_dtd(const char* symbol, const T* d_src, int nb_elt){
    CUDA_SAFE_CALL(cudaMemcpyToSymbol(symbol,
                                      reinterpret_cast<const void*>(d_src),
                                      nb_elt * sizeof(T),
                                      0,
                                      cudaMemcpyHostToDevice) );
}

/// Safe memory copy host to 1D cudaArray
template <class T>
inline
void mem_cpy_1D_htd(cudaArray* dst, const T* src, int nb_elt){
    int data_size = sizeof(T) * nb_elt;
    CUDA_SAFE_CALL(cudaMemcpyToArray(dst, 0, 0, src, data_size, cudaMemcpyHostToDevice));
}

/// Safe memory copy host to 2D cudaArray
template <class T>
inline
void mem_cpy_2D_htd(cudaArray* dst, const T* src, int2 nb_elt){
    int data_size = sizeof(T) * nb_elt.x * nb_elt.y;
    CUDA_SAFE_CALL(cudaMemcpyToArray(dst, 0, 0, src, data_size, cudaMemcpyHostToDevice));
}

/// Safe memory copy host to 3D cudaArray
template <class T>
inline
void mem_cpy_3D_htd(cudaArray* dst, const T* src, int3 nb_elt){
    cudaExtent volumeSize = make_cudaExtent(nb_elt.x, nb_elt.y, nb_elt.z);
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr = make_cudaPitchedPtr(reinterpret_cast<void*>(src),
                                            volumeSize.width*sizeof(T),
                                            volumeSize.width,
                                            volumeSize.height);
    copyParams.dstArray = dst;
    copyParams.extent   = volumeSize;
    copyParams.kind     = cudaMemcpyHostToDevice;
    CUDA_SAFE_CALL( cudaMemcpy3D(&copyParams) );
}

/// Safe allocation on device memory
template <class T>
inline
void malloc_d(T*& data, int nb_elt)
{
    if(nb_elt > 0)
        CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data),
                                  nb_elt * sizeof(T)));
    else
        data = 0;
}

/// Safe allocation on host memory
template <class T, bool page_lock>
inline
void malloc_h(T*& data, int nb_elt)
{
    if(nb_elt > 0)
    {
        if(page_lock)
        {
            CUDA_SAFE_CALL(cudaMallocHost(reinterpret_cast<void**>(&data),
                                          nb_elt * sizeof(T)));
        }
        else
            data = new T[nb_elt];
    }
    else
        data = 0;
}

/// Safe allocation of a 1D cudaArray
template<class T>
inline
void malloc_1D_array(cudaArray*& d_data, int nb_elt)
{
    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMallocArray(&d_data, &cfd, nb_elt, 1));
}

/// Safe allocation of a 2D cudaArray
template<class T>
inline
void malloc_2D_array(cudaArray*& d_data, int2 nb_elt)
{
    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMallocArray(&d_data, &cfd, nb_elt.x, nb_elt.y));
}

/// Safe allocation of a 3D cudaArray
template<class T>
inline
void malloc_3D_array(cudaArray*& d_data, int3 nb_elt)
{
    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<T>();
    cudaExtent volumeSize = make_cudaExtent(nb_elt.x, nb_elt.y, nb_elt.z);
    CUDA_SAFE_CALL(cudaMalloc3DArray(&d_data, &cfd, volumeSize) );
}

/// Safe memory deallocation on device
template <class T>
inline
void free_d(T*& data)
{
    CUDA_SAFE_CALL( cudaFree(reinterpret_cast<void*>(data)) );
    data = 0;
}

/// Safe memory deallocation of cuda arrays
template <>
inline
void free_d<cudaArray>(cudaArray*& data)
{
    CUDA_SAFE_CALL( cudaFreeArray(data) );
    data = 0;
}

/// Safe memory deallocation on host
template <class T, bool page_lock>
inline
void free_h(T*& data)
{
    if( page_lock ) CUDA_SAFE_CALL( cudaFreeHost(reinterpret_cast<void*>(data)) );
    else            delete[] data;

    data = 0;
}


}
// END CUDA_UTILS NAMESPACE ====================================================

#endif // CUDA_UTILS_COMMON_HPP__

#ifndef CUDA_UTILS__
#define CUDA_UTILS__

#if !defined(NO_CUDA)
#include <stdio.h>
#include <cuda.h>
#include <cassert>

#include "cuda_utils_common.hpp"
#include "vec3_cu.hpp"
#include "point_cu.hpp"
#include "cuda_utils_host_array.hpp"
#include "cuda_utils_device_array.hpp"
#include "cuda_utils_device_elt.hpp"

// Work around a nasty CUDA bug.  If any files in the application are compiled by nvcc, but don't
// contain any __device__ symbols, the CUDA debugger doesn't see any symbols in the whole application.
// Work around this by defining a dummy function in each file.  Make sure this file is included in
// all .cu files that may have no __device__ functions.
namespace {
    __device__ void fix_debug() { };

    // Avoid unused local warnings by referencing fix_debug.  We can't just use a variable,
    // since that variable would then be unused.  Declare a class, since compilers won't warn
    // about unreferenced class instances.
    struct Dummy {
        Dummy(void (*func)()) { }
    };
    Dummy d(fix_debug);
}

/**
    @namespace Cuda_utils
    @brief This file defines classes to handle both Host and Device arrays.

    The copy constructor in those classes only copy the pointers. This means
    that device arrays can be used as kernel arguments. To do an hard copy
    between arrays you will have to use explicitly copy_from() methods which
    allows copy from any combinaison of device/host arrays. Note that the
    operator '=' is forbiden (private).

    These arrays are freed automatically at their destruction

    You can easily copy data from device to host or any other order :

    @code
    // allocate 100 elements on host memory
    Cuda_utils::HA_int h_my_array(100);
    // allocate 100 elements on device memory
    Cuda_utils::DA_int d_my_array(100);

    ...

    // Copy host array in device array
    d_my_array.copy_from(h_my_array);
    // Every data flux is allowed :
    h_my_array.copy_from(d_my_array);
    d_my_array.copy_from(d_my_array);
    h_my_array.copy_from(h_my_array);

    // Note that you can also copy from another array type
    HA_float     3f_array(3);
    HA_Vec3_cu vec_array(1);
    ...
    3f_array.copy_from(vec_array);

    // More complex data movements can be done through Cuda_utils::mem_cpy_xxx()
    // functions family. But no special overflow underflow check will be done
    mem_cpy_htd(d_array.ptr()    + offset1,
                h_my_array.ptr() + offset2,
                h_my_array.size()-2
               );

    // Call a kernel with a device array will only copy the pointer
    cuda_kernel<<<1,1>>>(d_my_array);
    @endcode
*/

// =============================================================================
namespace Cuda_utils{
// =============================================================================

/// @return the device Id with the maximum compute capabilitie
int get_max_gflops_device_id();

/// print a device attributes (Max threads/blocks or texture sizes etc.)
void print_device_attribs(CUdevice id);

/// Memory usage of the GPU in megabytes
void get_device_memory_usage(double& free, double& total);

/**
  @class HD_Array

  @brief Array holding both device and host array

  Because often we need to maintain both device and host copies this class
  holds an host array and a device array.

  At construction both host and device array are allocated.

  Typical usage:
  @code
  // Allocating device and host memory
  HD_Array<int> hd_array(2);

  // doing stuff on host memory
  hd_array[0] = 3;
  hd_array.erase(1);
  hd_array.realloc(3);

  // reporting on device memory the changes
  hd_array.update_device_mem();

  // Launching kernel on the array
  ker<<< b, g>>>(hd_array.d_ptr(), hd_array.size());

  // reporting on host memory the changes
  hd_array.update_host_mem();

  @endcode
*/
template <class T>
struct HD_Array : public Host::Array<T> {

    HD_Array() : Host::Array<T>() { }

    /// Allocate both device and host memory
    HD_Array(int nb_elt) :
        Host::Array<T>(nb_elt),
        _d_array(nb_elt)
    {   }

    /// Allocate both device and host memory
    HD_Array(int nb_elt, const T &elt) :
        Host::Array<T>(nb_elt, elt),
        _d_array(nb_elt, elt)
    {   }

    /// Erase device memory with host memory
    /// Set both device/host arrays ith element to 'elt'
    void set_hd(int i, const T& elt){
        (*this)[i] = elt;
        _d_array.set(i, elt);
    }

    /// Copy a segment of the host mem into device
    void update_device_mem(int start, int nb_elt)
    {
        assert(Host::Array<T>::size() == _d_array.size());
        mem_cpy_htd( d_ptr() + start, Host::Array<T>::ptr() + start, nb_elt );
    }

    /// Copy a segment of the device mem into host
    void update_host_mem(int start, int nb_elt)
    {
        assert(Host::Array<T>::size() == _d_array.size());
        mem_cpy_dth( Host::Array<T>::ptr() + start, d_ptr() + start, nb_elt );
    }

    /// Overide all device memory with host memory.
    /// Device is automatically reallocated to match host size if needed
    void update_device_mem()
    {
        if(CCA::nb_elt == 0){
            _d_array.erase();
            return;
        }

        if(_d_array.size() != CCA::nb_elt)
            _d_array.malloc( CCA::nb_elt );

        _d_array.copy_from( *this );
    }

    /// Overide all host memory with the device memory
    /// Host is automatically reallocated to match device size if needed
    void update_host_mem()
    {
        if( _d_array.size() == 0){
            this->erase();
            return;
        }

        if(_d_array.size() != CCA::nb_elt)
            this->malloc( _d_array.size() );

        this->copy_from( _d_array);
    }

    /// @return a pointeur to the device memory
    T* d_ptr() { return _d_array.ptr(); }

    /// @return reference to the device array
    Device::Array<T>& device_array(){ return _d_array; }

    const Device::Array<T>& device_array() const{ return _d_array; }

private:
    typedef Cuda_utils::Common::Array<T> CCA;
    typedef Host::Array<T> CHA;
    Device::Array<T> _d_array;
};

// -----------------------------------------------------------------------------

/// @name Typenames shortcuts for common types
/// @{
typedef Device::Array<int> DA_int;
typedef Device::Array<bool> DA_bool;
typedef Device::Array<int2> DA_int2;
typedef Device::Array<int3> DA_int3;
typedef Device::Array<int4> DA_int4;
typedef Device::Array<unsigned> DA_uint;
typedef Device::Array<uint2> DA_uint2;
typedef Device::Array<uint3> DA_uint3;
typedef Device::Array<uint4> DA_uint4;
typedef Device::Array<float > DA_float;
typedef Device::Array<float2> DA_float2;
typedef Device::Array<float3> DA_float3;
typedef Device::Array<float4> DA_float4;
typedef Device::Array<Vec3_cu> DA_Vec3_cu;
typedef Device::Array<Point_cu> DA_Point_cu;

typedef Host::Array<int> HA_int;
typedef Host::Array<bool> HA_bool;
typedef Host::Array<int2> HA_int2;
typedef Host::Array<int3> HA_int3;
typedef Host::Array<int4> HA_int4;
typedef Host::Array<unsigned> HA_uint;
typedef Host::Array<uint2> HA_uint2;
typedef Host::Array<uint3> HA_uint3;
typedef Host::Array<uint4> HA_uint4;
typedef Host::Array<float > HA_float;
typedef Host::Array<float2> HA_float2;
typedef Host::Array<float3> HA_float3;
typedef Host::Array<float4> HA_float4;
typedef Host::Array<Vec3_cu> HA_Vec3_cu;
typedef Host::Array<Point_cu> HA_Point_cu;

typedef Host::PL_Array<int> HAPL_int;
typedef Host::PL_Array<bool> HAPL_bool;
typedef Host::PL_Array<int2> HAPL_int2;
typedef Host::PL_Array<int3> HAPL_int3;
typedef Host::PL_Array<int4> HAPL_int4;
typedef Host::PL_Array<int > HAPL_uint;
typedef Host::PL_Array<uint2> HAPL_uint2;
typedef Host::PL_Array<uint3> HAPL_uint3;
typedef Host::PL_Array<uint4> HAPL_uint4;
typedef Host::PL_Array<float > HAPL_float;
typedef Host::PL_Array<float2> HAPL_float2;
typedef Host::PL_Array<float3> HAPL_float3;
typedef Host::PL_Array<float4> HAPL_float4;
typedef Host::PL_Array<Vec3_cu> HAPL_Vec3_cu;
typedef Host::PL_Array<Point_cu> HAPL_Point_cu;

typedef HD_Array<int> HDA_int;
typedef HD_Array<bool> HDA_bool;
typedef HD_Array<int2> HDA_int2;
typedef HD_Array<int3> HDA_int3;
typedef HD_Array<int4> HDA_int4;
typedef HD_Array<int > HDA_uint;
typedef HD_Array<uint2> HDA_uint2;
typedef HD_Array<uint3> HDA_uint3;
typedef HD_Array<uint4> HDA_uint4;
typedef HD_Array<float > HDA_float;
typedef HD_Array<float2> HDA_float2;
typedef HD_Array<float3> HDA_float3;
typedef HD_Array<float4> HDA_float4;
typedef HD_Array<Vec3_cu> HDA_Vec3_cu;
typedef HD_Array<Point_cu> HDA_Point_cu;
/// @}

}// END CUDA_UTILS NAMESPACE ====================================================

#endif

#endif // CUDA_UTILS__

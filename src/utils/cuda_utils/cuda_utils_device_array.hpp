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
#ifndef DEVICE_ARRAY_HPP__
#define DEVICE_ARRAY_HPP__

//#include "cuda_utils_common.hpp"
#include "cuda_compiler_interop.hpp"
#include <vector>
#include <iostream>

/** @namespace Cuda_utils::Device
    @brief utilities to work on device memory with CUDA
    This file is part of the Cuda_utils homemade toolkit it handles device
    arrays

    @see Cuda_utils
*/

// =============================================================================
namespace Cuda_utils{
// =============================================================================

namespace Host{
template <class T, bool page_locked>
struct ArrayTemplate;
}

// =============================================================================
namespace Device{
// =============================================================================

/// @brief class of arrays on device global memory
/// @tparam T : type of the array data
template <class T>
struct Array : Cuda_utils::Common::Array<T>{

    template <class B>
    friend struct Cuda_utils::Device::Array;

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------
    IF_CUDA_DEVICE_HOST
    inline Array(): CCA(), data(0), state(0) { }

    /// @warning this implicit copy constructor only copy pointers
    IF_CUDA_DEVICE_HOST
    inline Array(const Array& d_a);

    /// Create from a user allocated pointer
    /// @param auto_free specify whether the memory should be freed at
    /// destruction the destruction of the array or not
    IF_CUDA_DEVICE_HOST
    inline Array(T* ptr, int nb_elt, bool auto_free);

    /// Allocate sizeof(T)*nb_elt into device memory the array
    inline Array(int nb_elt);

    /// Initialize and allocate sizeof(T)*nb_elt into device memory the array
    /// @param elt : element to fill the array with.
    inline Array(int nb_elt, const T& elt);

    inline ~Array();

    // -------------------------------------------------------------------------
    /// @name Methods
    // -------------------------------------------------------------------------

    /// Allocation or reallocation (always erase data)
    inline void malloc(int nb_elt);

    /// Allocation or reallocation (keep previous data)
    inline void realloc(int nb_elt);

    /// Erase the ith element
    /// @warning slow method (array is entirely duplicated)
    inline void erase(int i);

    /// Erase elements from the index start to the index end
    /// (start and end included)
    /// @warning slow method (array is entirely duplicated)
    inline void erase(int start, int end);

    /// Erase the array (memory is freed and array size == 0)
    /// @remarks If allocated the array is also freed with the destructor
    inline void erase();

    /// insert an host value in device memory at the index 'i'
    /// (insertion at the end is done with: d_a.insert(d_a.size(), elt)
    /// @warning slow method (array is entirely duplicated).
    void insert(int i, const T& val);

    /// insert host values in device memory at the index 'i'
    /// @note (insertion at the end is done with: d_a.insert(d_a.size(), elt_array)
    /// @warning slow method (array is entirely duplicated).
    /// @{
    void insert(int i, const Device::Array<T>& d_a);

    template <bool pg_lk>
    void insert(int i, const Host::ArrayTemplate<T, pg_lk>& h_a);

    void insert(int i, const std::vector<T>& h_vec);
    /// @}

    /// Copy from another array
    /// @return 0 if succeeded
    //@{
    template <class B, bool pg_lk>
    inline void copy_from(const Host::ArrayTemplate<B, pg_lk>& h_a);

    template <class B>
    inline void copy_from(const std::vector<B>& h_vec);

    template <class B>
    inline void copy_from(const Device::Array<B>& d_a);
    //@}


    inline       T* ptr()     { return data; }
    inline const T* ptr()const{ return data; }

    /// reinterpret the array data
    template <class B>
    IF_CUDA_DEVICE_HOST
    inline Cuda_utils::Device::Array<B> as_array_of() const;

    /// fetch value at index i from device memory with host code
    inline T fetch(int i) const;

    /// fetch value at index i from device memory with host code
    inline void fetch(int i, T& var) const;

    /// set value from host code into device memory
    /// @warning slow method
    inline void set(int i, const T& var);

    /// swap this array pointer and attributes with the given array
    inline void swap(Array& d);

    // For convenience and debugging: copy the array to host memory, and return it as a vector.
    inline std::vector<T> to_host_vector() const;

    #ifdef __CUDACC__
    /// access to array elements
    /// @warning only possible from device code
    __device__ __host__
    inline const T& operator[](int i) const {
        FORBID_HOST_CALL();
        return data[i];
    }

    /// access to array elements
    /// @warning only possible from device code
    __device__ __host__
    inline T& operator[](int i) {
        FORBID_HOST_CALL();
        return data[i];
    }

    /// Bind a device array to a linear texture
    /// @return if the array as been binded
    template <class B>
    bool bind_tex(texture<B, 1, cudaReadModeElementType>& tex_ref);
    #endif

private:
    /// @warning assignment operator: forbidden to use (use swap or copy from)
    inline Array& operator=(const Array& a) {
        assert(false);
        //*this = a.as_array_of<T>();

        Cuda_utils::Device::Array<T> res;
        res.state = a.state & CCA::IS_COPY;
        res.data = a.data;
        res.Cuda_utils::Common::Array<T>::nb_elt = a.CCA::nb_elt;
        return *this;
    }

    T* data;
    int state;
    typedef Cuda_utils::Common::Array<T> CCA;
};
// END ARRAY CLASS _____________________________________________________________


/**
 * @class CuArray
 * @brief Utility to allocate/copy 'cudaArray' type
 *
 * When using cuda textures one must use "cudaArray*" if linear interpolation
 * is to be used. This class provide utilities for allocating/copying data
 * of the cudaArray format.
 *
 * use case:
 * @code
 *
 * texture<float, 2, cudaReadModeElementType>  data_tex;
 * CuArray<float> d_data;
 *
 * {
 *     Host::Array<float> h_data(width * height, 0.f);
 *
 *     d_data.malloc(width, height):
 *     d_data.copy_from(h_data);
 *
 *     data_tex.normalized = false;
 *     data_tex.addressMode[0] = cudaAddressModeClamp;
 *     data_tex.addressMode[1] = cudaAddressModeClamp;
 *     data_tex.filterMode = cudaFilterModeLinear;
 *     d_data.bind_tex( data_tex );
 *     // Texture is ready to use with lerp
 * }
 *
 * @endcode
 *
 */
template <class T>
struct CuArray : Cuda_utils::Common::Array<T>{

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------
    inline CuArray() :
        CCA(),
        data(0),
        state(0),
        cuda_flags(0),
        array_extent(make_cudaExtent(0,0,0))
    { }

    /// Recopy only copy pointers
    inline CuArray(const CuArray& ca) :
        CCA(ca.nb_elt),
        data(ca.data),
        state(ca.state | CCA::IS_COPY),
        cuda_flags(ca.cuda_flags),
        array_extent(make_cudaExtent(0,0,0))
    { }

    template<bool pg_lk>
    explicit inline CuArray(const Cuda_utils::Host::ArrayTemplate<T,pg_lk>& h_a);

    explicit inline CuArray(const Cuda_utils::Device::Array<T>& d_a);

    explicit inline CuArray(int dimx);
    explicit inline CuArray(int dimx, int dimy);
    explicit inline CuArray(int dimx, int dimy, int dimz);

    //Destructor
    inline ~CuArray();

    // -------------------------------------------------------------------------
    /// @name Allocation (always erase data)
    // -------------------------------------------------------------------------
    /// @{
    inline void malloc(int dimx, int dimy=0, int dimz=0);
    /// @}

    /// Free device memory
    inline void erase();

    // -------------------------------------------------------------------------
    /// @name Copy
    // -------------------------------------------------------------------------

    /// Copy from another host array
    /// @return 0 if succeeded
    template <class B, bool pg_lk>
    inline int copy_from(const Cuda_utils::Host::ArrayTemplate<B, pg_lk>& h_a);

    /// upload data from device memory to the CuArray
    /// @param d_a : the device array in the standard global memory space
    /// @param size : number of elements of the array d_a.
    /// @return 0 if succeeded
    template <class B>
    inline int copy_from(B* d_a, int size);

    inline cudaExtent get_extent() const { return array_extent; }
    inline int size() const { return array_extent.width * array_extent.height * array_extent.depth; }

    inline void set_cuda_flags(int flags) {
        // Do this before allocating.
        assert(state == 0);
        cuda_flags = flags;
    }

    // -------------------------------------------------------------------------
    /// @name Methods
    // -------------------------------------------------------------------------
    #ifdef __CUDACC__
    /// @warning don't forget to setup the texture paremeters
    /// (clamping filter mode etc.)
    template <int dim>
    inline void bind_tex(texture<T, dim, cudaReadModeElementType>& texref) const {
        if(CCA::nb_elt > 0) CUDA_SAFE_CALL(cudaBindTextureToArray(texref, data));
    }

    cudaArray *getCudaArray() { return data; }
    const cudaArray *getCudaArray() const { return data; }
    #endif

private:

    /// @warning assignment operator: forbidden to use (use copy from)
    inline CuArray& operator=(const CuArray& a) {
        return a;
    }

    cudaArray* data;
    int state;
    int cuda_flags;
    cudaExtent array_extent;
    typedef Cuda_utils::Common::Array<T> CCA;
};
// END CUARRAY CLASS ___________________________________________________________

}
// END DEVICE NAMESPACE ========================================================

}
// END CUDA_UTILS NAMESPACE ====================================================

////////////////////////////////////////////////////////////////////////////////
//Device array methods
////////////////////////////////////////////////////////////////////////////////

template <class T>
inline Cuda_utils::Device::Array<T>::

Array(int nb_elt) :
    CCA(nb_elt),
    state(CCA::IS_ALLOCATED)
{
    data = 0;
    CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data), nb_elt * sizeof(T)));
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::Array<T>::

Array(int nb_elt, const T& elt) :
    CCA(nb_elt),
    state(CCA::IS_ALLOCATED)
{
    data = 0;
    CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data), nb_elt * sizeof(T)));

    // Fill the array:
    std::vector<T> vec(nb_elt, elt);
    this->copy_from(vec);
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::Array<T>::

Array(const Cuda_utils::Device::Array<T>& d_a) :
    CCA(d_a.nb_elt),
    data(d_a.data),
    state(d_a.state | CCA::IS_COPY)
{ /*       */ }

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::Array<T>::

Array(T* ptr, int nb_elt, bool auto_free) :
    CCA(nb_elt),
    data(ptr)
{
    state = CCA::IS_ALLOCATED;
    if(!auto_free)
        state =  state | CCA::IS_COPY;
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::Array<T>::

~Array()
{
    if( (state & CCA::IS_ALLOCATED) && !(state & CCA::IS_COPY) && (CCA::nb_elt > 0) )
    {
        CUDA_SAFE_CALL(cudaFree(data));
        data = 0;
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

malloc(int nb_elt)
{
    if(!(state & CCA::IS_ALLOCATED)){
        CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data), nb_elt * sizeof(T)));
        state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);
        CCA::nb_elt = nb_elt;
    } else {
        if(state & CCA::IS_COPY){
            CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data), nb_elt * sizeof(T)));
            state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);
            CCA::nb_elt = nb_elt;
        } else {
            if(nb_elt == CCA::nb_elt) return;
            CUDA_SAFE_CALL(cudaFree(data));
            data = 0;
            CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data), nb_elt * sizeof(T)));
            state = (state | CCA::IS_ALLOCATED);
            CCA::nb_elt = nb_elt;
        }
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

realloc(int nb_elt)
{
    if(!(state & CCA::IS_COPY))
    {
        if(nb_elt == CCA::nb_elt) return;

        T* data_tmp;
        CUDA_SAFE_CALL(cudaMalloc(reinterpret_cast<void**>(&data_tmp), nb_elt * sizeof(T)));
        CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data_tmp),
                                  reinterpret_cast<const void*>(data),
                                  (nb_elt > CCA::nb_elt ? CCA::nb_elt : nb_elt) * sizeof(T),
                                  cudaMemcpyDeviceToDevice));
        if(state & CCA::IS_ALLOCATED){
            CUDA_SAFE_CALL(cudaFree(data));
            data = 0;
        }
        data = data_tmp;
        CCA::nb_elt = nb_elt;
        state = CCA::IS_ALLOCATED;
    }else
        fprintf(stderr,"cuda_utils : Can't realloc an implicit copy !\n");
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

erase(int start, int end)
{
    assert(start >= 0);
    assert(start <= end);
    assert( end < CCA::size() );

    if( state & CCA::IS_ALLOCATED )
    {
        const int nb_elt = end-start+1;
        Array<T> tmp(CCA::size()-nb_elt);
        mem_cpy_dtd(tmp.data        , data      , start            );
        mem_cpy_dtd(tmp.data + start, data+end+1, CCA::size()-end-1);
        tmp.swap(*this);
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

erase(int i)
{
    erase(i, i);
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

erase()
{
    if((state & CCA::IS_ALLOCATED) & !(state & CCA::IS_COPY))
    {
        CUDA_SAFE_CALL(cudaFree(data));
        data  = 0;
        state = 0;
        CCA::nb_elt = 0;
    }
}

// -----------------------------------------------------------------------------

template <class T>
template <bool pg_lk>
inline void Cuda_utils::Device::Array<T>::

insert(int i, const Host::ArrayTemplate<T, pg_lk>& h_a)
{
    assert(i >= 0);
    assert(i <= CCA::size());
    if(h_a.size() != 0)
    {
        Array<T> tmp(CCA::size() + h_a.size() );
        mem_cpy_dtd(tmp.data               , data          , i            );
        mem_cpy_htd(tmp.data + i           , h_a.ptr(), h_a.size()   );
        mem_cpy_dtd(tmp.data + i+h_a.size(), data + i      , CCA::size()-i);
        tmp.swap(*this);
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

insert(int i, const std::vector<T>& h_vec)
{
    assert(i >= 0);
    assert(i <= CCA::size());
    if( h_vec.size() != 0)
    {
        Array<T> tmp(CCA::size() + h_vec.size() );
        mem_cpy_dtd(tmp.data                 , data     , i             );
        mem_cpy_htd(tmp.data + i             , &h_vec[0], h_vec.size()  );
        mem_cpy_dtd(tmp.data + i+h_vec.size(), data + i , CCA::size()-i );
        tmp.swap(*this);
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

insert(int i, const Device::Array<T>& d_a)
{
    assert(i >= 0);
    assert(i <= CCA::size());
    if(d_a.size() != 0)
    {
        Array<T> tmp(CCA::size() + d_a.size() );
        mem_cpy_dtd(tmp.data               , data          , i            );
        mem_cpy_dtd(tmp.data + i           , d_a.ptr(), d_a.size()   );
        mem_cpy_dtd(tmp.data + i+d_a.size(), data + i      , CCA::size()-i);
        tmp.swap(*this);
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

insert(int i, const T& val)
{
    assert(i >= 0);
    assert(i <= CCA::size());

    Array<T> tmp(CCA::size() + 1 );
    mem_cpy_dtd(tmp.data      , data     , i             );
    mem_cpy_htd(tmp.data + i  , &val     , 1             );
    mem_cpy_dtd(tmp.data + i+1, data + i , CCA::size()-i );
    tmp.swap(*this);

}

// -----------------------------------------------------------------------------

template <class T>
template <class B, bool pg_lk>
inline void Cuda_utils::Device::Array<T>::

copy_from(const Host::ArrayTemplate<B, pg_lk>& h_a)
{
    assert(state & CCA::IS_ALLOCATED);

    int bytes_to_copy;
    if(CCA::nb_elt * sizeof(T) >= h_a.size() * sizeof(B))
        bytes_to_copy = h_a.size() * sizeof(B);
    else
    {
        bytes_to_copy = CCA::nb_elt * sizeof(T);
        fprintf(stderr,"cuda_utils : warning array capacity exceeded\n");
        assert(false);
    }

    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
                              reinterpret_cast<const void*>(h_a.ptr()),
                              bytes_to_copy,
                              cudaMemcpyHostToDevice));

}

// -----------------------------------------------------------------------------

template<typename A, typename B>
inline void cuda_mem_cpy_from(A* dst, const std::vector<B> &src, int bytes_to_copy){
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(dst),
                              reinterpret_cast<const void*>(&src[0]),
                              bytes_to_copy,
                              cudaMemcpyHostToDevice));
}

template<>
inline void cuda_mem_cpy_from< bool, bool >(bool* dst, const std::vector<bool>& src, int bytes_to_copy){
    bool *tmp = new bool[src.size()];
    for (unsigned int i=0; i<src.size(); ++i)
        tmp[i] = src[i];

    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(dst),
                              reinterpret_cast<const void*>(&tmp[0]),
                              bytes_to_copy,
                              cudaMemcpyHostToDevice));
    delete[] tmp;
}

// -----------------------------------------------------------------------------

template <class T>
template <class B>
inline void Cuda_utils::Device::Array<T>::

copy_from(const std::vector<B>& h_vec)
{
    assert(state & CCA::IS_ALLOCATED);
    if(h_vec.size() > 0)
    {
        int bytes_to_copy;
        if(CCA::nb_elt * sizeof(T) >= h_vec.size() * sizeof(B))
            bytes_to_copy = h_vec.size() * sizeof(B);
        else
        {
            bytes_to_copy = CCA::nb_elt * sizeof(T);
            fprintf(stderr,"cuda_utils : warning array capacity exceeded\n");
            assert(false);
        }

        cuda_mem_cpy_from(data, h_vec, bytes_to_copy);
//        CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
//                                  reinterpret_cast<const void*>(&tmp[0]),
//                                  bytes_to_copy,
//                                  cudaMemcpyHostToDevice));
    }
}

// -----------------------------------------------------------------------------

template <class T>
template <class B>
inline void Cuda_utils::Device::Array<T>::

copy_from(const Device::Array<B>& d_a)
{
    assert(state & CCA::IS_ALLOCATED);

    int bytes_to_copy;
    if(CCA::nb_elt * sizeof(T) >= d_a.size() * sizeof(B))
        bytes_to_copy = d_a.size() * sizeof(B);
    else
    {
        bytes_to_copy = CCA::nb_elt * sizeof(T);
        fprintf(stderr,"cuda_utils : warning array capacity exceeded\n");
        assert(false);
    }

    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
                              reinterpret_cast<const void*>(d_a.ptr()),
                              bytes_to_copy,
                              cudaMemcpyDeviceToDevice));

}

// -----------------------------------------------------------------------------

#ifdef __CUDACC__
template <class T>
template <class B>
bool Cuda_utils::Device::Array<T>::

bind_tex(texture<B, 1, cudaReadModeElementType>& tex_ref)
{
    int size = CCA::nb_elt;
    if(size > 0)
    {
        tex_ref.addressMode[0] = cudaAddressModeWrap;
        tex_ref.addressMode[1] = cudaAddressModeWrap;
        tex_ref.filterMode = cudaFilterModePoint;
        tex_ref.normalized = false;
        CUDA_SAFE_CALL(cudaBindTexture(0,
                                       tex_ref,
                                       data,
                                       size * sizeof(T))
                       );
        return true;
    }
    return false;
}
#endif

// -----------------------------------------------------------------------------

template <class T>
inline T Cuda_utils::Device::Array<T>::

fetch(int i) const
{
    assert(i >= 0);
    assert(i < CCA::nb_elt);
//    struct To_POD{
//        char __c[sizeof(T)];
//        inline operator T() const {return *reinterpret_cast<const T*>(__c);}
//    } fdata;
    T fdata;
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(&fdata),
                              reinterpret_cast<const void*>(&(data[i])),
                              sizeof(T),
                              cudaMemcpyDeviceToHost));
    return fdata;
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

fetch(int i, T& var) const
{
    assert(i >= 0);
    assert(i < CCA::nb_elt);
    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(&var),
                              reinterpret_cast<const void*>(&(data[i])),
                              sizeof(T),
                              cudaMemcpyDeviceToHost));
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

set(int i, const T& var)
{
    assert(i < CCA::nb_elt);
    mem_cpy_htd(data+i, &var, 1);
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::Array<T>::

swap(Array& d)
{
    T* data_tmp = data;
    int state_tmp = state;
    int nb_tmp = CCA::nb_elt;
    data = d.data;
    state = d.state;
    CCA::nb_elt = d.nb_elt;
    d.data = data_tmp;
    d.state = state_tmp;
    d.nb_elt = nb_tmp;
}


template <class T>
inline std::vector<T> Cuda_utils::Device::Array<T>::
to_host_vector() const
{
    Cuda_utils::Host::Array<T> host_array;
    host_array.malloc(size());
    host_array.copy_from(*this);
    return std::vector<T>(&host_array[0], &host_array[0] + size());
}

// -----------------------------------------------------------------------------

template <class T>
template <class B>
inline Cuda_utils::Device::Array<B> Cuda_utils::Device::Array<T>::

as_array_of() const
{
    Cuda_utils::Device::Array<B> res;
    res.state = state & CCA::IS_COPY;
    res.data = reinterpret_cast<B*>(data);
    res.Cuda_utils::Common::Array<B>::nb_elt = (CCA::nb_elt * sizeof(T) )/sizeof(B);
    return res;
}

////////////////////////////////////////////////////////////////////////////////
// CuArray methods
////////////////////////////////////////////////////////////////////////////////

template <class T>
template<bool pg_lk>
inline Cuda_utils::Device::CuArray<T>::

CuArray(const Cuda_utils::Host::ArrayTemplate<T,pg_lk>& h_a):
    CCA(h_a.size()),
    state(CCA::IS_ALLOCATED),
    array_extent(make_cudaExtent(h_a.size(),0,0))
{
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMalloc3DArray(&data, &channelDesc, array_extent));
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr  = make_cudaPitchedPtr(reinterpret_cast<void*>(h_a.ptr()),
                                             array_extent.width*sizeof(T),
                                             array_extent.width, array_extent.height);
    copyParams.dstArray= data;
    copyParams.extent  = array_extent;
    copyParams.kind    = cudaMemcpyHostToDevice;
    CUDA_SAFE_CALL( cudaMemcpy3D(&copyParams) );
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::CuArray<T>::

CuArray(const Cuda_utils::Device::Array<T>& d_a):
    CCA(d_a.size()),
    state(CCA::IS_ALLOCATED),
    array_extent(make_cudaExtent(d_a.size(),0,0))
{
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMalloc3DArray(&data, &channelDesc, array_extent, cuda_flags));
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr  = make_cudaPitchedPtr(reinterpret_cast<void*>(d_a.ptr()),
                                             array_extent.width*sizeof(T),
                                             array_extent.width, array_extent.height);
    copyParams.dstArray= data;
    copyParams.extent  = array_extent;
    copyParams.kind    = cudaMemcpyDeviceToDevice;
    CUDA_SAFE_CALL( cudaMemcpy3D(&copyParams) );
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::CuArray<T>::

CuArray(int dimx):
    CCA(dimx),
    state(CCA::IS_ALLOCATED),
    array_extent(make_cudaExtent(dimx, 0, 0))
{
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMalloc3DArray(&data, &channelDesc, array_extent, cuda_flags));
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::CuArray<T>::

CuArray(int dimx, int dimy):
    CCA(dimx*dimy),
    state(CCA::IS_ALLOCATED),
    array_extent(make_cudaExtent(dimx, dimy, 0))
{
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMalloc3DArray(&data, &channelDesc, array_extent, cuda_flags));
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::CuArray<T>::

CuArray(int dimx, int dimy, int dimz):
    CCA(dimx*dimy*dimz),
    state(CCA::IS_ALLOCATED),
    array_extent(make_cudaExtent(dimx, dimy, dimz))
{
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<T>();
    CUDA_SAFE_CALL(cudaMalloc3DArray(&data, &channelDesc, array_extent, cuda_flags));
}

// -----------------------------------------------------------------------------

template <class T>
inline Cuda_utils::Device::CuArray<T>::

~CuArray()
{
    if((state & CCA::IS_ALLOCATED) & !(state & CCA::IS_COPY) & CCA::nb_elt > 0){
        CUDA_SAFE_CALL(cudaFreeArray(data));
        data = 0;
    }
}

// -----------------------------------------------------------------------------

template <class T>
inline void Cuda_utils::Device::CuArray<T>::

erase()
{
    if((state & CCA::IS_ALLOCATED) & !(state & CCA::IS_COPY) & CCA::nb_elt > 0)
    {
        CUDA_SAFE_CALL(cudaFreeArray(data));
        data  = 0;
        state = 0;
        CCA::nb_elt = 0;
    }
}

template <class T>
inline void Cuda_utils::Device::CuArray<T>::malloc(int dimx, int dimy, int dimz)
{
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<T>();
    array_extent = make_cudaExtent(dimx, dimy, dimz);

    int nb_elt = dimx;
    if(dimy != 0)
        nb_elt *= dimy;
    if(dimz != 0)
        nb_elt *= dimz;

    assert(nb_elt >= 0);

    if((state & CCA::IS_ALLOCATED) && !(state & CCA::IS_COPY)){
        CUDA_SAFE_CALL(cudaFreeArray(data));
        data = NULL;
    }
    state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);

    CUDA_SAFE_CALL(cudaMalloc3DArray(&data, &channelDesc, array_extent, cuda_flags));
    CCA::nb_elt = nb_elt;
}

// -----------------------------------------------------------------------------

template <class T>
template <class B, bool pg_lk>
inline int Cuda_utils::Device::CuArray<T>::

copy_from(const Cuda_utils::Host::ArrayTemplate<B, pg_lk>& h_a)
{
    assert(CCA::nb_elt * sizeof(T) == h_a.size() * sizeof(B));
    if((state & CCA::IS_ALLOCATED) && h_a.size() > 0)
    {
        cudaMemcpy3DParms copyParams = {0};
        copyParams.srcPtr  = make_cudaPitchedPtr(const_cast<void*>(reinterpret_cast<const void*>(h_a.ptr())), // const_cast is so ugly I know ...
                                                 array_extent.width*sizeof(T),
                                                 array_extent.width,
                                                 array_extent.height);
        copyParams.dstArray= data;
        copyParams.extent  = array_extent;
        copyParams.kind    = cudaMemcpyHostToDevice;
        CUDA_SAFE_CALL(cudaMemcpy3D(&copyParams));
        return 0;
    }
    return 1;
}

// -----------------------------------------------------------------------------

template <class T>
template <class B>
inline int Cuda_utils::Device::CuArray<T>::

copy_from(B* d_a, int size)
{
    assert(CCA::nb_elt * sizeof(T) == size * sizeof(B));
    if((state & CCA::IS_ALLOCATED) && size > 0)
    {
        cudaMemcpy3DParms copyParams = {0};
        copyParams.srcPtr  = make_cudaPitchedPtr(d_a,
                                                 array_extent.width*sizeof(T),
                                                 array_extent.width,
                                                 array_extent.height);
        copyParams.dstArray= data;
        copyParams.extent  = array_extent;
        copyParams.kind    = cudaMemcpyDeviceToDevice;
        CUDA_SAFE_CALL(cudaMemcpy3D(&copyParams));
        return 0;
    }
    return 1;
}

// -----------------------------------------------------------------------------


#endif // DEVICE_ARRAY_HPP__

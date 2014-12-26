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
#ifndef HOST_ARRAY_HPP__
#define HOST_ARRAY_HPP__

//#include "cuda_utils_common.hpp"
#include "cuda_compiler_interop.hpp"
#include <vector>

/** @namespace Cuda_utils::Host
    @brief utilities to work on host memory with CUDA
    This file is part of the Cuda_utils homemade toolkit it handles host arrays

    @see Cuda_utils
*/

// type of a host array of data T and page lock state x
#define PL_TYPE(T,x) typename Cuda_utils::Host::PL_Type<T,x>::Type

// =============================================================================
namespace Cuda_utils{
// =============================================================================

namespace Device{
template <class T>
struct Array;
}

// =============================================================================
namespace Host{
// =============================================================================

template <class T>
struct Array;

template <class T>
struct PL_Array;

template<class T, bool b>
struct PL_Type{
    typedef PL_Array<T> Type;
};

template<class T>
struct PL_Type<T,false>{
    typedef Array<T> Type;
};


/** @brief class of arrays on host memory
    @tparam T : type of the array data
    @tparam  page lock : true if handled by cuda (faster memory transfers to
    and from the device. However, page lock memory is scarce so use it wisely.
    Excessive use of page lock memory could result in poor performance)

    @warning Do not try using this class. Instead use Host::Array for
    standard host arrays and Host::PL_Array for page_locked arrays
*/
template <class T, bool page_locked>
struct ArrayTemplate : Cuda_utils::Common::Array<T>{

    virtual ~ArrayTemplate();

    template <class B, bool pg_lk>
    friend struct Cuda_utils::Host::ArrayTemplate;

    /// Allocation or reallocation (always erase data)
    inline void malloc(int nb_elt);

    /// Allocation or reallocation (always erase data)
    /// @param elt : the element to fill the array with
    inline void malloc(int nb_elt, const T& elt);

    /// Fill the array with the element elt
    inline void fill(const T& elt);

    /// Allocation or reallocation (keep previous data)
    inline void realloc(int nb_elt);

    /// Erase the ith element
    /// @warning could be slow (array is entirely duplicated)
    inline void erase(int i);

    /// Erase elements from the index start to the index end
    /// (start and end included)
    /// @warning slow method (array is entirely duplicated)
    inline void erase(int start, int end);

    /// insert a value at the index 'i'
    /// (insert an element at the end is done by h_a.insert(h_a.size(), elt)
    /// @warning slow method (array is entirely duplicated).
    void insert(int i, const T& val);

    /// insert values at the index 'i'
    /// (insert an element at the end is done by h_a.insert(h_a.size(), elt)
    /// @warning slow method (array is entirely duplicated).
    //@{
    void insert(int i, const Device::Array<T>& d_a);

    template <bool pg_lk>
    void insert(int i, const Host::ArrayTemplate<T, pg_lk>& h_a);

    void insert(int i, const std::vector<T>& h_vec);
    //@}

    /// Erase the array
    void erase();

    /// Copy from another array
    /// returns 0 if succeeded
    /// @note the array we copy from is not necessarily of the same type
    //@{
    template <class B, bool pg_lk>
    inline void copy_from(const ArrayTemplate<B, pg_lk>& h_a);

    template <class B>
    inline void copy_from(const std::vector<B>& h_vec);

    template <class B>
    inline void copy_from(const B* h_vec, int nb_elt);

    template <class B>
    inline void copy_from(const Cuda_utils::Device::Array<B>& d_a);
    //@}

    inline       T* ptr()       { return data; }
    inline const T* ptr() const { return data; }

    /// access to array elements
    /// only possible from the host
    inline const T& operator[](int i) const {
        assert(i < CCA::nb_elt);
        assert(i >= 0);
        return data[i];
    }

    inline T& operator[](int i){
        assert(i < CCA::nb_elt);
        assert(i >= 0);
        return data[i];
    }

    /// reinterpret the array data
    template <class B>
    IF_CUDA_DEVICE_HOST
    inline PL_TYPE(B,page_locked) as_array_of() const;

    /// Swap arrays of same element types
    inline void swap(PL_TYPE(T,page_locked)& d);

protected:

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------
    inline ArrayTemplate(): CCA(), data(0), state(0) {}

    // copy constructor only copy pointers
    inline ArrayTemplate(const ArrayTemplate& h_a) :
        CCA(h_a.nb_elt),
        data(h_a.data),
        state(h_a.state | CCA::IS_COPY)
    {    }

    inline ArrayTemplate(int nb_elt);


    inline ArrayTemplate(int nb_elt, const T& elt);

private:
    T* data;
    int state;
    typedef Cuda_utils::Common::Array<T> CCA;

    /// @warning assignment operator is forbidden
    /// (instead use swap() or copy_from())
    inline ArrayTemplate& operator=(const ArrayTemplate& a) {return *this;}
};
//  END ArrayTemplate CLASS ____________________________________________________


/// @brief class of non page-locked arrays on host memory
/// shortcut class using ArrayTemplate<T, false>
/// @tparam T : type of the array data
template <class T>
struct Array : ArrayTemplate<T,false>{

    inline Array(): BaseT() {}

    template<class ArrayT>
    inline Array(const ArrayT& h_a): BaseT(h_a) {}

    inline Array(int nb_elt): BaseT(nb_elt) {}

    inline Array(int nb_elt, const T& elt): BaseT(nb_elt, elt) {}

    virtual ~Array(){}

private:
    typedef ArrayTemplate<T,false> BaseT;
};
// END ARRAY CLASS _____________________________________________________________

/// @brief class of page-locked arrays on host memory
/// shortcut class using ArrayTemplate<T, true>
/// @tparam  T : type of the array data
template <class T>
struct PL_Array : ArrayTemplate<T,true>{

    inline PL_Array(): BaseT() {}

    //implicit copy constructor
    template<class ArrayT>
    inline PL_Array(const ArrayT& h_a): BaseT(h_a) {}

    inline PL_Array(int nb_elt): BaseT(nb_elt) {}

    inline PL_Array(int nb_elt, const T& elt): BaseT(nb_elt, elt) {}

    virtual inline ~PL_Array(){}

private:
    typedef ArrayTemplate<T,true> BaseT;
};
// END PL_Array CLASS __________________________________________________________

}
// END HOST NAMESPACE ==========================================================

}
// END CUDA_UTILS NAMESPACE ====================================================

////////////////////////////////////////////////////////////////////////////////
// Host array methods
////////////////////////////////////////////////////////////////////////////////

template <class T, bool page_locked>
inline Cuda_utils::Host::ArrayTemplate<T,page_locked>::

ArrayTemplate(int nb_elt) :
    CCA(nb_elt),
    data(0),
    state(CCA::IS_ALLOCATED)
{
    assert(nb_elt >= 0);

    malloc_h<T, page_locked>(data, nb_elt);
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline Cuda_utils::Host::ArrayTemplate<T,page_locked>::

ArrayTemplate(int nb_elt, const T& elt) :
    CCA(nb_elt),
    data(0),
    state(CCA::IS_ALLOCATED)
{
    assert(nb_elt >= 0);

    malloc_h<T, page_locked>(data, nb_elt);
    fill(elt);
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline Cuda_utils::Host::ArrayTemplate<T,page_locked>::

~ArrayTemplate()
{
    if((state & CCA::IS_ALLOCATED) & !(state & CCA::IS_COPY))
        free_h<T, page_locked>(data);
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

malloc(int nb_elt)
{
    assert(nb_elt >= 0);
    if(!(state & CCA::IS_ALLOCATED))
    {
        malloc_h<T, page_locked>(data, nb_elt);
        state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);
        CCA::nb_elt = nb_elt;
    }
    else
    {
        if(state & CCA::IS_COPY)
        {
            malloc_h<T, page_locked>(data, nb_elt);
            state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);
            CCA::nb_elt = nb_elt;
        }
        else
        {
            free_h<T, page_locked>(data);
            malloc_h<T, page_locked>(data, nb_elt);
            state = (state | CCA::IS_ALLOCATED);
            CCA::nb_elt = nb_elt;
        }
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

malloc(int nb_elt, const T& elt)
{
    assert(nb_elt >= 0);
    if(!(state & CCA::IS_ALLOCATED))
    {
        malloc_h<T, page_locked>(data, nb_elt);
        state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);
        CCA::nb_elt = nb_elt;
    }
    else
    {
        if(state & CCA::IS_COPY)
        {
            malloc_h<T, page_locked>(data, nb_elt);
            state = (state | CCA::IS_ALLOCATED) & (~CCA::IS_COPY);
            CCA::nb_elt = nb_elt;
        }
        else
        {
            free_h<T, page_locked>(data);
            malloc_h<T, page_locked>(data, nb_elt);
            state = (state | CCA::IS_ALLOCATED);
            CCA::nb_elt = nb_elt;
        }
    }
    fill(elt);
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

fill(const T& elt)
{
    for (int i = 0; i < CCA::nb_elt; ++i)
        data[i] = elt;
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

realloc(int nb_elt)
{
    assert(nb_elt >= 0);
    if(!(state & CCA::IS_COPY))
    {
        if(nb_elt == CCA::nb_elt) return;

        T* data_tmp;
        malloc_h<T, page_locked>(data_tmp, nb_elt);

        CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data_tmp),
                                  reinterpret_cast<const void*>(data),
                                  (nb_elt > CCA::nb_elt ? CCA::nb_elt : nb_elt) * sizeof(T),
                                  cudaMemcpyHostToHost));

        if(state & CCA::IS_ALLOCATED)
            free_h<T, page_locked>(data);

        data = data_tmp;
        CCA::nb_elt = nb_elt;
        state = CCA::IS_ALLOCATED;
    }else
        fprintf(stderr,"cuda_utils : Can't realloc an implicit copy !\n");
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

erase(int i)
{
    erase(i, i);
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

erase(int start, int end)
{
    assert(start >= 0);
    assert(start <= end);
    assert( end < CCA::size() );
    if( state & CCA::IS_ALLOCATED )
    {
        T*  data_tmp;
        const int nb_elt = end-start+1;
        int data_tmp_size = CCA::size()-nb_elt;

        malloc_h<T, page_locked>(data_tmp, data_tmp_size);

        mem_cpy_hth(data_tmp        , data    , start              );
        mem_cpy_hth(data_tmp + start, data+end+1, CCA::size()-end-1);

        T* tmp      = data;
        data        = data_tmp;
        CCA::nb_elt = data_tmp_size;

        free_h<T, page_locked>(tmp);
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

insert(int i, const Device::Array<T>& d_a)
{
    assert(i >= 0);
    assert(i <= CCA::size());
    if(d_a.size() != 0)
    {
        int new_size = CCA::size() + d_a.size();

        T* tmp;
        malloc_h<T, page_locked>(tmp, new_size);

        mem_cpy_hth(tmp               , data          , i            );
        mem_cpy_dth(tmp + i           , d_a.ptr(), d_a.size()   );
        mem_cpy_hth(tmp + i+d_a.size(), data + i      , CCA::size()-i);

        T* old      = data;
        data        = tmp;
        CCA::nb_elt = new_size;

        free_h<T, page_locked>(old);

        state = (state | CCA::IS_ALLOCATED);
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

insert(int i, const std::vector<T>& h_vec)
{
    assert(i >= 0);
    assert(i <= CCA::size());
    if(h_vec.size() != 0)
    {
        int new_size = CCA::size() + h_vec.size();

        T* tmp;
        malloc_h<T, page_locked>(tmp, new_size);

        mem_cpy_hth(tmp                 , data     , i             );
        mem_cpy_hth(tmp + i             , &h_vec[0], h_vec.size()  );
        mem_cpy_hth(tmp + i+h_vec.size(), data + i , CCA::size()-i );

        T* old      = data;
        data        = tmp;
        CCA::nb_elt = new_size;

        free_h<T, page_locked>(old);

        state = (state | CCA::IS_ALLOCATED);
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
template <bool pg_lk>
void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

insert(int i, const Host::ArrayTemplate<T, pg_lk>& h_a)
{
    assert(i >= 0);
    assert(i <= CCA::size());
    if(h_a.size() != 0)
    {
        int new_size = CCA::size() + h_a.size();

        T* tmp;
        malloc_h<T, page_locked>(tmp, new_size);

        mem_cpy_hth(tmp               , data          , i            );
        mem_cpy_hth(tmp + i           , h_a.ptr(), h_a.size()   );
        mem_cpy_hth(tmp + i+h_a.size(), data + i      , CCA::size()-i);

        T* old      = data;
        data        = tmp;
        CCA::nb_elt = new_size;

        free_h<T, page_locked>(old);
        state = (state | CCA::IS_ALLOCATED);
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

insert(int i, const T& val)
{
    assert(i >= 0);
    assert(i <= CCA::size());

    int new_size = CCA::size()+1;

    T* tmp;
    malloc_h<T, page_locked>(tmp, new_size);

    mem_cpy_hth(tmp               , data    , i            );
    tmp[i] = val;
    mem_cpy_hth(tmp + i+1         , data + i, CCA::size()-i);

    T* old      = data;
    data        = tmp;
    CCA::nb_elt = new_size;

    free_h<T, page_locked>(old);
    state = (state | CCA::IS_ALLOCATED);
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

erase()
{
    if((state & CCA::IS_ALLOCATED) & !(state & CCA::IS_COPY))
    {
        free_h<T, page_locked>(data);
        state = 0;
        CCA::nb_elt = 0;
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
template <class B, bool pg_lk>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

copy_from( const ArrayTemplate<B, pg_lk>& h_a )
{
    assert(state & CCA::IS_ALLOCATED);

    int bytes_to_copy;
    if(CCA::nb_elt * sizeof(T) >= h_a.size() * sizeof(B))
        bytes_to_copy = h_a.size() * sizeof(B);
    else
    {
        bytes_to_copy  = CCA::nb_elt * sizeof(T);
        fprintf(stderr,"cuda_utils : host : warning array capacity exceeded\n");
        assert(false);
    }

    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
                              reinterpret_cast<const void*>(h_a.ptr()),
                              bytes_to_copy,
                              cudaMemcpyHostToHost));

}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
template <class B>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

copy_from(const std::vector<B>& h_vec)
{
    assert(state & CCA::IS_ALLOCATED);
    if( h_vec.size() > 0 )
    {
        int bytes_to_copy;
        if(CCA::nb_elt * sizeof(T) >= h_vec.size() * sizeof(B))
            bytes_to_copy = h_vec.size() * sizeof(B);
        else
        {
            bytes_to_copy  = CCA::nb_elt * sizeof(T);
            fprintf(stderr,"cuda_utils: host : warning array capacity exceeded\n");
            assert(false);
        }

        CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
                                  reinterpret_cast<const void*>(&h_vec[0]),
                                  bytes_to_copy,
                                  cudaMemcpyHostToHost));
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
template <class B>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

copy_from(const B* h_ptr, int nb_elt)
{
    assert(state & CCA::IS_ALLOCATED);
    if( nb_elt > 0 )
    {
        int bytes_to_copy;
        if(CCA::nb_elt * sizeof(T) >= nb_elt * sizeof(B))
            bytes_to_copy = nb_elt * sizeof(B);
        else
        {
            bytes_to_copy  = CCA::nb_elt * sizeof(T);
            fprintf(stderr,"cuda_utils: host : warning array capacity exceeded\n");
            assert(false);
        }

        CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
                                  reinterpret_cast<const void*>( h_ptr ),
                                  bytes_to_copy,
                                  cudaMemcpyHostToHost));
    }
}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
template <class B>
inline void Cuda_utils::Host::ArrayTemplate<T,page_locked>::

copy_from( const Cuda_utils::Device::Array<B>& d_a )
{
    assert(state & CCA::IS_ALLOCATED);

    int bytes_to_copy;
    if(CCA::nb_elt * sizeof(T) >= d_a.size() * sizeof(B))
        bytes_to_copy = d_a.size() * sizeof(B);
    else
    {
        bytes_to_copy = (CCA::nb_elt * sizeof(T));
        fprintf(stderr,"cuda_utils : host : warning array capacity exceeded\n");
        assert(false);
    }

    CUDA_SAFE_CALL(cudaMemcpy(reinterpret_cast<void*>(data),
                              reinterpret_cast<const void*>(d_a.ptr()),
                              bytes_to_copy,
                              cudaMemcpyDeviceToHost));

}

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
inline void Cuda_utils::Host::ArrayTemplate<T, page_locked>::

swap(PL_TYPE(T,page_locked)& d)
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

// -----------------------------------------------------------------------------

template <class T, bool page_locked>
template <class B>
inline PL_TYPE(B,page_locked) Cuda_utils::Host::ArrayTemplate<T, page_locked>::

as_array_of() const
{
    PL_TYPE(B,page_locked) res;
    res.state = state & CCA::IS_COPY;
    res.data  = reinterpret_cast<B*>(data);
    res.Cuda_utils::Common::Array<B>::nb_elt = (CCA::nb_elt * sizeof(T) )/sizeof(B);
    return res;
}


#endif // HOST_ARRAY_HPP__

#ifndef DEVICE_ELT_HPP__
#define DEVICE_ELT_HPP__

//#include "cuda_utils_common.hpp"
#include "macros.hpp"
#include <vector>
#include <iostream>

// =============================================================================
namespace Cuda_utils{
// =============================================================================

// =============================================================================
namespace Device{
// =============================================================================

/// @class Elt
/// @brief Automatic upload of an element of type 'T' to GPU
template <class T>
struct Elt {

    Elt(const T& elt) {
        malloc_d( _d_ptr, 1);
        mem_cpy_htd(_d_ptr, elt, 1);
    }

    ~Elt() { free_d(_d_ptr); }

    T* ptr(){ return _d_ptr; }

private:
    T* _d_ptr;
};

}// END Device =================================================================

}// END Cuda_utils =============================================================

#endif //DEVICE_ELT_HPP__

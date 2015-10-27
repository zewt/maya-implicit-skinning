#ifndef HRBF_PHI_FUNCS_HPP_
#define HRBF_PHI_FUNCS_HPP_

#include "math_cu.hpp"
#include "cuda_compiler_interop.hpp"

/** @brief Radial basis functions definitions (function phi)
  Here you can add more radial basis function definitions.
  To enable the RBF wrapper to use it, one has to change the typedef of PHI_TYPE
  located in rebf_setup.hpp
*/

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

// =============================================================================
template<typename Scalar>
struct Rbf_pow3
{
    IF_CUDA_DEVICE_HOST
    static inline Scalar f  (const Scalar& x) { return x*x*x;             }
    IF_CUDA_DEVICE_HOST
    static inline Scalar df (const Scalar& x) { return Scalar(3) * x * x; }
    IF_CUDA_DEVICE_HOST
    static inline Scalar ddf(const Scalar& x) { return Scalar(6) * x;     }
};

// =============================================================================

template<typename Scalar>
struct Rbf_x_sqrt_x
{
    IF_CUDA_DEVICE_HOST
    static inline Scalar f  (const Scalar& x2) {
        Scalar x = std::sqrt(x2); return x*x*x;
    }
    IF_CUDA_DEVICE_HOST
    static inline Scalar df (const Scalar& x2) {
        return Scalar(3)/Scalar(2) * std::sqrt(x2);
    }
    IF_CUDA_DEVICE_HOST
    static inline Scalar ddf(const Scalar& x2) {
        return Scalar(3)/Scalar(4) / std::sqrt(x2);
    }
};

// =============================================================================


// thin plates
// take care of adjusting thin plates evaluation in hermiteRBF.inl in methods f, gf and fngf

#define THIN_PLATES_ORDER_x_2 2

template<typename Scalar>
struct Rbf_thin_plate
{
    IF_CUDA_DEVICE_HOST
    static inline Scalar f (const Scalar& x) {
        return pow( x, THIN_PLATES_ORDER_x_2 ) * log(x);
    }
    IF_CUDA_DEVICE_HOST
    static inline Scalar df (const Scalar& x) {
        return pow( x, THIN_PLATES_ORDER_x_2 - Scalar(1) ) * ( THIN_PLATES_ORDER_x_2 * log(x) + Scalar(1) );
    }
    IF_CUDA_DEVICE_HOST
    static inline Scalar ddf(const Scalar& x) {
        return pow( x, THIN_PLATES_ORDER_x_2 - Scalar(2) ) * ( THIN_PLATES_ORDER_x_2 * (THIN_PLATES_ORDER_x_2 - Scalar(1)) * log(x)
                                                               + THIN_PLATES_ORDER_x_2 - Scalar(1) + THIN_PLATES_ORDER_x_2 );
    }
};

// =============================================================================

}// END RBF_wrapper ============================================================

#endif //HRBF_PHI_FUNCS_HPP_

#ifndef BLENDING_FUNCTIONS_HPP__
#define BLENDING_FUNCTIONS_HPP__

#include "uclean.hpp"
#include "ucommon.hpp"
#include "ultimate.hpp"
#include "dyn_operators.hpp"


/** @brief Blending operators for implicit surfaces

  operators must follow this implementation :

  @code

struct Binnary_operator{
    __device__ __host__
    static float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ __host__
    static Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ __host__
    static float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);
};

   @endcode


*/

#endif // BLENDING_FUNCTIONS_HPP__

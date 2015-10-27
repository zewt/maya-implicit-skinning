#include "vec3_cu.hpp"

// =============================================================================
namespace Field{
// =============================================================================

#define DISTANCE_OFFSET 0.3f

template <int n>
IF_CUDA_DEVICE_HOST
inline float pow(float a){
    float b = pow<n/2>(a);
    if(n & 1){
        return a * b * b;
    } else {
        return b * b;
    }
}

template <>
IF_CUDA_DEVICE_HOST
inline float pow<1>(float a){
    return a;
}

template <>
IF_CUDA_DEVICE_HOST
inline float pow<0>(float){
    return 1.f;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float distance_squared_to_field(float d2, float radius){
#ifdef LARGE_SUPPORT_PRIMITIVES
    float x = fminf(sqrtf(d2)/radius, 1.f);
    return 1.f - sqrtf(x);
#else
    float f = 1.f - fminf(d2/(radius * radius), 1.f);
    return pow<ALPHA>(f);
#endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float distance_to_field(float d, float radius){
    return distance_squared_to_field(d * d, radius);
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float field_derivative_from_distance(float d, float radius){
#ifdef LARGE_SUPPORT_PRIMITIVES
    return -0.5f / sqrtf(radius * d);
#else
    float d2 = d * d;
    float f = 1.f - fminf(d2 / (radius * radius), 1.f);
    return -(2 * ALPHA) *  d * pow<ALPHA-1>(f);
#endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float2 distance_to_field_and_derivative(float d, float radius){
#ifdef LARGE_SUPPORT_PRIMITIVES
    float x = sqrtf(fminf(d / radius, 1.f));
    return make_float2(1.f - x, -0.5f / (radius * x));
#else
    float d2 = d * d;
    float f = 1.f - fminf(d2 / (radius * radius), 1.f);
    float pf = pow<ALPHA-1>(f);
    return make_float2(pf * f, -(2 * ALPHA) *  d * pf);
#endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float field_to_distance(float f, float radius){
#ifdef LARGE_SUPPORT_PRIMITIVES
    float df = 1.f - f;
    return radius * df * df;
#else
    return radius * sqrtf(1.f - powf(f, 1.f / ALPHA));
#endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float distance_to_field_flatten(float d, float radius, float offset){
    if (d < radius-offset)
        return 1.f;
    if (d > radius+offset)
        return 0.f;
    float u = (d-(radius-offset)) / (2*offset);
    float u3 = u*u*u, u2 = u*u;
    return 2*u3 - 3*u2 + 1;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float field_derivative_from_distance_flatten(float d, float radius, float offset){
    if (d < radius-offset || d > radius+offset)
        return 0.f;

    float u = (d-(radius-offset)) / (2*offset);
    return 6*u*u - 6u;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float2 distance_to_field_and_derivative_flatten(float d, float radius, float offset){
    if (d < radius-offset)
        return make_float2(1.f, 0.f);
    if (d > radius+offset)
        return make_float2(0.f, 0.f);

    float u = (d-(radius-offset)) / (2*offset);
    float u3 = u*u*u, u2 = u*u;
    return make_float2(2*u3 - 3*u2 + 1, 6*u*u - 6u);
}

// -----------------------------------------------------------------------------

struct Flatten_field{
    static double f(double x){
        return (double)distance_to_field_flatten((float)x, r, o);
    }
    static float r, o;
};

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float to_compact_poly_c1(float f, float radius)
{
    if(f < -radius)
        return 1.f;
    else if(f > radius)
        return 0.f;
    else
    {
        // (1/4) * (f/radius)^3 - (3/4) * (f/radius) + 1/2
        const float fact = (f/radius);
        return (1.f/4.f)*pow<3>(fact) - (3.f/4.f)*fact + (1.f/2.f);
    }
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline void grad_to_compact_poly_c1(float f, float r, Vec3_cu& grad)
{
    if(f < -r || f > r)
        grad = Vec3_cu(0.f,0.f,0.f);
    else
    {
        // (3/(4r)) * (f/radius)^2 - (3/(4r))
        const float fact  = 3.f/(4.f*r);
        const float scale = fact*pow<2>(f/r) - fact;
        grad = grad * scale;
    }
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float to_compact_poly_c2(float f, float radius)
{
    if(f < -radius)
        return 1.f;
    else if(f > radius)
        return 0.f;
    else
    {
        // (-3/16)*(f/radius)^5+(5/8)*(f/radius)^3-(15/16)*(f/radius) + 1/2
        const float fact   = (f/radius);
        const float fact2  = fact  * fact;
        const float fact4  = fact2 * fact2;
        return (-3.f/16.f)*fact4*fact + (5.f/8.f)*fact2*fact - (15.f/16.f)*fact + 1.f/2.f;
    }

}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline void grad_to_compact_poly_c2(float f, float r, Vec3_cu& grad)
{
    if(f < -r || f > r)
        grad = Vec3_cu(0.f,0.f,0.f);
    else
    {
        // (-15/(16r))*(f/r)^4 + (15/(8*r))*(f/r)^2 - (15/(16r))
        const float fact  = f/r;
        const float fact2 = fact*fact;
        const float fact4 = fact2*fact2;
        const float tmp   = (-15.f/(16.f*r));
        const float scale = tmp*fact4 + (15.f/(8.f*r))*fact2 + tmp;
        grad = grad * scale;
    }
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline float to_compact_tanh(float f, float radius, float to)
{
    if(f < -radius)
        return 1.f;
    else if(f > radius)
        return 0.f;
    else
    {
        // exp(f*(-to/radius)) / (exp(f*(-to/radius))+1)
        const float exp = expf(f*(-to/radius));
        return exp / (exp+1.f);
    }

}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST
inline void grad_to_compact_tanh(float f, float r, float to, Vec3_cu& grad)
{
    if(f < -r || f > r)
        grad = Vec3_cu(0.f,0.f,0.f);
    else
    {
        // [  (-to/r)*exp(x*(-to/r))^2 + (-to/r)*exp(x*(-to/r)) + (to/r) * exp(x*(-to/r))^2 ] / (exp(x*(-to/r))+1)^2
        const float exp   = expf(f*(-to/r));
        const float exp2  = exp * exp;
        const float fact  =	to/r;
        const float scale = (-fact*exp2-fact*exp+fact*exp2)/pow<2>(exp+1);
        grad = grad * scale;
    }
}

}
// END FIELD NAMESPACE =========================================================

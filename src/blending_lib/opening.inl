#include "opening.hpp"

#include <iostream>
#include <cassert>
#include <algorithm>
//#include <cmath>
#include "math_cu.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// =============================================================================
namespace IBL {
// =============================================================================

// =============================================================================
namespace Opening {
// =============================================================================

IF_CUDA_DEVICE_HOST inline
Pan_hf::Pan_hf()
{
    FORBID_DEVICE_CALL();
    #ifndef __CUDA_ARCH__
        init_samples();
    #endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
float Pan_hf::linear_fetch(float t) const
{
    FORBID_DEVICE_CALL();
    #ifndef __CUDA_ARCH__
    t = std::max(0.f, std::min(2.f, t));
    float u = t * 0.5f * (_nb_samples - 1);
    int i = (int)floor(u);
    float dt = u - (float)i;
    if(dt == 0.f){
        assert(i<_nb_samples);
        return (float)_vals[i];
    }
    assert(i+1<_nb_samples);
    float v0 = (float)_vals[i    ];
    float v1 = (float)_vals[i + 1];
    return v1 * dt + v0 * (1.f - dt);
    #else
    return 0.f;
    #endif
}

// -----------------------------------------------------------------------------

template<class Data>
IF_CUDA_DEVICE_HOST inline
Discreet_hyperbola_base<Data>::

Discreet_hyperbola_base(Curve_t type) : _type(type) { }

// -----------------------------------------------------------------------------

template<class Data>
IF_CUDA_DEVICE_HOST inline
float Discreet_hyperbola_base<Data>::f(float x, float tan_alpha) const
{
    if(_type == OPEN_TANH)
        return open_pan_hyperbola(x, tan_alpha);

    if ( x <= 1.f)
        return open_pan_hyperbola(x, tan_alpha);
    else if (x > 2.f)
        return 2.f;
    else
    {
        switch(_type) {
        case CLOSED_HERMITE:
        {
            return hermite_closure(x, tan_alpha);
        }
        case CLOSED_TANH:
        {
            return tanh_closure(x, tan_alpha);
        }
        default:
            // Undefined type
            assert(false);
            return 0.f;
        }
    }
}


// -----------------------------------------------------------------------------

template<class Data> inline
float Discreet_hyperbola_base<Data>::

open_pan_hyperbola(float t, float tan_alpha) const
{
    float z;
    if(t <= tan_alpha){
        z = t;
    } else {
        float dt = 2.f * (t - tan_alpha)/(1.f-tan_alpha);
        z =  _pan_hf.linear_fetch(dt)*0.5f*(1.f - tan_alpha) + tan_alpha;
    }
    z *= (2.f / (1.f +tan_alpha));
    return z*z*tan_alpha;
}

// -----------------------------------------------------------------------------

template<class Data> inline
float Discreet_hyperbola_base<Data>::

hermite_closure(float x, float tan_alpha) const
{
    float a = 0.f, b = 1.f, u = 0.5f;
    float f1, f2, f3, f4;
    for(int i = 0; i < 15; ++i)
    {
        f1 = 2.f*u*u*u - 3*u*u + 1.f;
        f2 = -2.f*u*u*u + 3.f*u*u;
        f3 = u*u*u - 2.f*u*u + u;
        if ( f1 + 2.f*f2 + f3*(3.f-2.f*tan_alpha) > x ){
            b = u;
            u = (a+b)/2.f;
        } else {
            a = u;
            u = (a+b)/2.f;
        }
    }
    f1 = 2.f*u*u*u - 3.f*u*u + 1.f;
    f2 = -2.f*u*u*u + 3.f*u*u;
    f4 = u*u*u - u*u;
    return 1.f*tan_alpha*f1 + 2.f*f2 + 0.f + f4*(2.f-tan_alpha);
}

// -----------------------------------------------------------------------------

template<class Data> inline
float Discreet_hyperbola_base<Data>::

tanh_closure(float x, float tan_alpha) const
{
    float a   = -tanh(1.f);
    float res = (-1.f/a * tanh(tanh(tan(M_PI/2  *(2  *(x-1  )/2  -1  )))) + 1  ) * (2  -tan_alpha) + tan_alpha;
    return res;
}

// -----------------------------------------------------------------------------

inline IF_CUDA_DEVICE_HOST
float Line::f(float x, float tan_alpha) const
{
    return tan_alpha*x;
}

// -----------------------------------------------------------------------------

inline IF_CUDA_DEVICE_HOST
float Diamond::f(float x, float tan_alpha) const
{

    float point_inter = _inner * 2.f;

    if( x < 0.5f*2.f){
        return tan_alpha*x;
    }
    else if( x >= 0.5f*2.f && x < point_inter)
    {
        float tan_x = tan_alpha * x;
        float slope = (point_inter - tan_x) / (point_inter - 0.5f*2.f);
        float b = point_inter - slope * point_inter;
        return slope * x + b;
    }
    else
        return x;
}


}
// End Opening =================================================================

}// END IBL ====================================================================

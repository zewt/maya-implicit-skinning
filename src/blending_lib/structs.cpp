#include "structs.hpp"

#include <cmath>

// =============================================================================
namespace IBL {
// =============================================================================

float clamp(float value, float minv, float maxv)
{
    float n_val = value;
    if(n_val > maxv) n_val = maxv;
    if(n_val < minv) n_val = minv;
    return n_val;
}

double2 make_double2(double x, double y)
{
    double2 s = {x, y};
    return s;
}

double dot(double2 a, double2 b)
{
    return a.x*b.x + a.y*b.y;
}

double2 normalized(double2 a)
{
    double norm = sqrt(a.x * a.x + a.y * a.y);
    return make_double2(a.x / norm, a.y / norm);
}

double2 mult(double2 a, double s)
{
    return make_double2(a.x * s, a.y * s);
}

float2 make_float2(float x, float y)
{
    float2 s = {x, y};
    return s;
}

float3 make_float3(float x, float y, float z)
{
    float3 s = {x, y, z};
    return s;
}

float dot(float2 a, float2 b)
{
    return a.x*b.x + a.y*b.y;
}

float dot(float3 a, float3 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

float2 normalized(float2 a)
{
    float norm = sqrtf(a.x * a.x + a.y * a.y);
    return make_float2(a.x / norm, a.y / norm);
}

float3 normalized(float3 a)
{
    float norm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    return make_float3(a.x / norm, a.y / norm, a.z / norm);
}

float2 mult(float2 a, float s)
{
    return make_float2(a.x * s, a.y * s);
}

float3 mult(float3 a, float s)
{
    return make_float3(a.x * s, a.y * s, a.z * s);
}

}
// END IBL =====================================================================

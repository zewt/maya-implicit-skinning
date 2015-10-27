#ifndef IBL_STRUCTS_HPP__
#define IBL_STRUCTS_HPP__

// =============================================================================
namespace IBL {
// =============================================================================

float clamp(float value, float minv, float maxv);

// -----------------------------------------------------------------------------

struct double2 { double x, y; };

double2 make_double2(double x, double y);

double dot(double2 a, double2 b);

double2 normalized(double2 a);

double2 mult(double2 a, double s);

// -----------------------------------------------------------------------------

struct float2 { float x, y; };

float2 make_float2(float x, float y);

float dot(float2 a, float2 b);

float2 normalized(float2 a);

float2 mult(float2 a, float s);

// -----------------------------------------------------------------------------

struct float3 { float x, y, z; };

float3 make_float3(float x, float y, float z);

float dot(float3 a, float3 b);

float3 mult(float3 a, float s);

}
// END IBL =====================================================================

#endif // IBL_STRUCTS_HPP__

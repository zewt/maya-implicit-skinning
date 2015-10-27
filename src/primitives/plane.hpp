#ifndef PLANE_HPP__
#define PLANE_HPP__

#include "point_cu.hpp"
#include "vec3_cu.hpp"
//#include "implicit_types.hpp"
#include "distance_field.hpp"

/// @brief implicit plane
struct Plane{
    __device__ __host__
    inline Plane() {};

    __device__ __host__
    inline Plane(const Vec3_cu& normal, const Point_cu& b, float r) :
        n(normal.normalized()), radius(r / Field::field_to_distance(0.5f,1.f)),
        offset(pt_vec_dot(b,n) - Field::field_to_distance(0.5f,r / Field::field_to_distance(0.5f,1.f))) {};

    __device__ __host__
    float	f(const Point_cu& p) const;

    __device__ __host__
    inline Vec3_cu	gf(const Point_cu& p) const;

    __device__ __host__
    inline float fngf(Vec3_cu& gf, const Point_cu& p) const;

    __device__ __host__
    static inline float pt_vec_dot(const Point_cu& p, const Vec3_cu& v);

private:
    Vec3_cu n;
    float radius;
    float offset;
};

#include "plane.inl"

#endif // PLANE_HPP__

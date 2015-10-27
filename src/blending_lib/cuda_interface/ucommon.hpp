#ifndef UCOMMON_HPP_
#define UCOMMON_HPP_

// =============================================================================

struct Usum{
    __device__ __host__ static inline
    float f(float f1, float f2, const Vec3_cu& , const Vec3_cu& ){
        return f1 + f2;
    }

    __device__ __host__ static inline
    Vec3_cu gf(float , float , const Vec3_cu& gf1, const Vec3_cu& gf2){
        return gf1 + gf2;
    }

    __device__ __host__ static inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        gf = gf1 + gf2;
        return f1 + f2;
    }
};

// =============================================================================

struct Umax{

    __device__ __host__ static inline
    float f(float f1, float f2, const Vec3_cu& , const Vec3_cu& ){
        return fmaxf(f1,f2);
    }

    __device__ __host__ static inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return (f1>f2)?gf1:gf2;
    }

    __device__ __host__ static  inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        if(f1 > f2){
            gf = gf1;
            return f1;
        } else {
            gf = gf2;
            return f2;
        }
    }
};

// =============================================================================

struct Ucircle{

    __device__ __host__ static  inline
    float f(float f1, float f2, const Vec3_cu& , const Vec3_cu& ){
        return sqrtf(f1*f1 + f2*f2);
    }

    __device__ __host__ static  inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float denom = sqrtf(f1 * f1 + f2 * f2);
        float sq_term = (denom > 0.f)? 1.f / denom : 0.f;
        return (gf1 * f1 + gf2 * f2) * sq_term;
    }

    __device__ __host__ static  inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float denom = sqrtf(f1 * f1 + f2 * f2);
        float sq_term = (denom > 0.f) ? 1.f / denom : 0.f;
        gf = (gf1 * f1 + gf2 * f2) * sq_term;
        return denom;
    }
};

// =============================================================================

template<class U>
struct Ifrom{

    __device__ __host__ static  inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return _not(U::f(_not(f1), _not(f2), -gf1, -gf2));
    }

    __device__ __host__ static  inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return -U::gf(_not(f1), _not(f2), -gf1, -gf2);
    }

    __device__ __host__ static  inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float res = _not(U::fngf(gf, _not(f1), _not(f2), -gf1, -gf2));
        gf = -gf;
        return res;
    }

    __device__ __host__ static inline
    float _not(float f){ return 1.f - f; }

};

// =============================================================================

typedef Ifrom<Umax> Umin;

// =============================================================================

template <class Uc, class Ub>
struct Ihybrid{

    __device__ __host__ static inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return Ifrom<Uc>::f(f1, f2, gf1, gf2);
    }

    __device__ __host__ static  inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return Ifrom<Ub>::gf(f1, f2, gf1, gf2);
    }

    __device__ __host__ static inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        gf = Ifrom<Ub>::gf(f1, f2, gf1, gf2);
        return f(f1, f2, gf1, gf2);
    }
};

#endif // UCOMMON_HPP_

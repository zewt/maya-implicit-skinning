#ifndef N_ARY_HPP
#define N_ARY_HPP

#include "vec3_cu.hpp"

// -----------------------------------------------------------------------------

__constant__ float RICCI_N;

// -----------------------------------------------------------------------------

struct URicci{
    __device__ static  inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return powf( powf(f1, RICCI_N) + powf(f2, RICCI_N) , 1.f/ RICCI_N);
    }

    __device__ static  inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        return gf1.normalized() * f1 + gf2.normalized() * f2;
    }

    __device__ static  inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        gf = gf1.normalized() * f1 + gf2.normalized() * f2;
        return powf( powf(f1, RICCI_N) + powf(f2, RICCI_N) , 1.f/ RICCI_N);
    }

};

// =============================================================================

__constant__ float a0;
__constant__ float a1;
__constant__ float gji;
__constant__ float w0;
__constant__ float w1;
__constant__ float gij;

#define PI 3.14159265

// -----------------------------------------------------------------------------

__device__
float propagation(float k, float a, float w, float r){
    if (r <= w/2){
        float c = 4*(w*k-4*a)/(w*w*w);
        float d = 4*(3*a-w*k)/(w*w);
        return c*r*r*r + d*r*r + k*r;
    } else if (r <= w){
        return (a0*0.5f + a0*0.5f*cos(2*PI/w * (r-w/2)));
    } else {
        return 0.f;
    }
}

// -----------------------------------------------------------------------------

struct CaniContactUnaire{
    __device__
    static float f(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        if (f2 > 0.5) // zone d'interpenetration
            return f1 + 0.5 - f2;
        return f1 + propagation(gf2.norm(), a0*gji, w0, 0.5-f2);

//        float gji = (f2>0.5f?0.5-f2:0);
//        if (f2 < 0.5)
//            return f1 + gji + propagation(gf2.norm(), a0*gji, w0, f2-0.5);
//        else
//            return f1 + gji;
    }

    __device__
    static Vec3_cu gf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        return gf1;
    }

    __device__
    static float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        gf = gf1;
        if (f2 > 0.5)
            return f1 + 0.5 - f2;
        return f1 + propagation(gf2.norm(), a0*gji, w0, 0.5-f2);

//        float gji = (f2>0.5f?0.5-f2:0);
//        gf = gf1;
//        if (f2 < 0.5)
//            return f1 + gji + propagation(gf2.norm(), a0*gji, w0, f2-0.5);
//        else
//            return f1 + gji;
    }
};

struct CaniContact{
    __device__
    static float f(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        float F1, F2;
        if (f2 > 0.5){ // zone d'interpenetration
            F1 = f1 + 0.5 - f2;
        } else {
            F1 = f1 + propagation(gf2.norm(), a0*gji, w0, 0.5-f2);
        }
        if (f1 > 0.5){
            F2 = f2 + 0.5 - f1;
        } else {
            F2 = f2 + propagation(gf1.norm(), a1*gij, w1, 0.5-f1);
        }
        return max(F1, F2);
//        float gji = (f2>0.5f?0.5-f2:0);
//        float gij = (f1>0.5f?0.5-f1:0);
//        float F1 = f1 + gji + propagation(gf2.norm(), a0*gji, w0, f2-0.5);
//        float F2 = f2 + gij + propagation(gf1.norm(), a1*gij, w1, f1-0.5);
//        return max(F1, F2);
    }

    __device__
    static Vec3_cu gf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        float F1, F2;
        if (f2 > 0.5){ // zone d'interpenetration
            F1 = f1 + 0.5 - f2;
        } else {
            F1 = f1 + propagation(gf2.norm(), a0*gji, w0, 0.5-f2);
        }
        if (f1 > 0.5){ // zone d'interpenetration
            F2 = f2 + 0.5 - f1;
        } else {
            F2 = f2 + propagation(gf1.norm(), a1*gij, w1, 0.5-f1);
        }
        if (F1 > F2)
            return gf1;
        else
            return gf2;
//        float gji = (f2>0.5f?0.5-f2:0);
//        float gij = (f1>0.5f?0.5-f1:0);
//        float F1 = f1 + gji + propagation(gf2.norm(), a0*gji, w0, f2-0.5);
//        float F2 = f2 + gij + propagation(gf1.norm(), a1*gij, w1, f1-0.5);
//        if (F1 > F2)
//            return gf1;
//        else
//            return gf2;
    }

    __device__
    static float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float F1, F2;
        if (f2 > 0.5){ // zone d'interpenetration
            F1 = f1 + 0.5 - f2;
        } else {
            F1 = f1 + propagation(gf2.norm(), a0*gji, w0, 0.5-f2);
        }
        if (f1 > 0.5){ // zone d'interpenetration
            F2 = f2 + 0.5 - f1;
        } else {
            F2 = f2 + propagation(gf1.norm(), a1*gij, w1, 0.5-f1);
        }
        if (F1 > F2){
            gf = gf1;
            return F1;
        } else {
            gf = gf2;
            return F2;
        }
//        float gji = (f2>0.5f?0.5-f2:0);
//        float gij = (f1>0.5f?0.5-f1:0);
//        float F1 = f1 + gji + propagation(gf2.norm(), a0*gji, w0, f2-0.5);
//        float F2 = f2 + gij + propagation(gf1.norm(), a1*gij, w1, f1-0.5);
//        if (F1 > F2){
//            gf = gf1;
//            return F1;
//        } else {
//            gf = gf2;
//            return F2;
//        }
    }
};

// =============================================================================

__constant__ float wA0A0;
__constant__ float wA0A1;
__constant__ float wA1A0;
__constant__ float wA1A1;

// -----------------------------------------------------------------------------

__device__
float sA0A1(float f, float w){
    if (2*f < 1)
        return 1+(4*f*f - 4*f)*(1-w);
    return w;
}

// -----------------------------------------------------------------------------

__device__
float kA0(float f, float w_auto, float w){
    return 0.5*(1-w_auto * sA0A1(f, w));
}

// -----------------------------------------------------------------------------

__device__
float mk( float t, float k ){
    if (t <= k)
        return 0.f;
    if (t >= 1-k)
        return 1.f;
    float k_t = k - t;
    float k_c = k - 0.5;
    float e_kt = 8*k*k - 12.5*k + 5 + 9*k*t - 7.5*t + 3*t*t;// using c = 0.5
    return (k_t*k_t*k_t)*e_kt / (16*k_c*k_c*k_c*k_c*k_c);
}

// -----------------------------------------------------------------------------

__device__
float mk_tilde(float t, float k){
    float s = (1-k*2);
    s *= s*s;
    return mk(t,k)*(1-s)+t*s;
}

// -----------------------------------------------------------------------------

struct RestrictedBlendUnaire{
    __device__
    static float f(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        return mk_tilde( f1, kA0(f2, wA0A0, wA0A1) );
    }

    __device__
    static Vec3_cu gf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        return gf1 * mk_tilde( f1, kA0(f2, wA0A0, wA0A1) );
    }

    __device__
    static float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float res = mk_tilde( f1, kA0(f2, wA0A0, wA0A1) );
        gf = gf1 * mk_tilde( f1, kA0(f2, wA0A0, wA0A1) );
        return res;
    }
};

// -----------------------------------------------------------------------------

struct RestrictedBlend{
    __device__
    static float f(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        return mk_tilde( f1, kA0(f2, wA0A0, wA0A1) ) +
               mk_tilde( f2, kA0(f1, wA1A1, wA1A0) );
    }

    __device__
    static Vec3_cu gf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2){
        return gf1 * mk_tilde( f1, kA0(f2, wA0A0, wA0A1) ) +
               gf2 * mk_tilde( f2, kA0(f1, wA1A1, wA1A0) );
    }

    __device__
    static float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float res1 = mk_tilde( f1, kA0(f2, wA0A0, wA0A1) );
        float res2 = mk_tilde( f2, kA0(f1, wA1A1, wA1A0) );
        gf = gf1 * res1 + gf2 * res2;
        return res1 + res2;
    }
};

// -----------------------------------------------------------------------------

// TODO: these parameters should not be in constant mem but rather attributes
// of their respective class.

// You should not need to add 'static keyword' as this header must be included
// only once
void init_nary_operators()
{
    float value = 1.f;
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA0A0, &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA0A1, &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA1A1, &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA1A0, &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(a0   , &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(w0   , &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(a1   , &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(w1   , &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    value = 0.f;
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(gji, &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
    CUDA_SAFE_CALL( cudaMemcpyToSymbol(gij, &value, sizeof(float), 0, cudaMemcpyHostToDevice) );
}

// -----------------------------------------------------------------------------

// =============================================================================
namespace N_ary {
// =============================================================================

void set_RICCI_N(float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(RICCI_N, &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }

void set_wA0A0(float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA0A0, &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_wA0A1(float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA0A1, &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_wA1A1(float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA1A1, &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_wA1A0(float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(wA1A0, &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_a0   (float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(a0   , &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_w0   (float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(w0   , &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_a1   (float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(a1   , &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_w1   (float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(w1   , &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_gji  (float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(gji  , &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }
void set_gij  (float v){ CUDA_SAFE_CALL( cudaMemcpyToSymbol(gij  , &v, sizeof(float), 0, cudaMemcpyHostToDevice) ); }

} // END N_ary =================================================================

#endif // N_ARY_HPP

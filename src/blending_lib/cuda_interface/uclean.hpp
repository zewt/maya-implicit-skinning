#ifndef UCLEAN_HPP_
#define UCLEAN_HPP_

/** @file ultimate.hpp
 *
 * @warning the following operators using cuda textures that must be bound
 * before being used
 * @see Blending_env
 */
#if !defined(BLENDING_ENV_HPP__)
#error "You must include blending_env.hpp before the inclusion of 'ultimate.hpp'"
#endif

#include "vec3_cu.hpp"
#include "ultimate.hpp"
#include "operator3d_cu.hpp"

#define RANGE (1.f)

// Activate computation of the potential for values outside [0 1] for OMUCircle
//#define HACK

#define X0 (0.75f)

/**
    @class OMUCircle Opennable max union in circles
    Barthe's arc of an ellipse controlled by Gourmel's gradient controller

  @warning the operator is precomputed for a potential between [0 1] outside
  values has undefined behavior.
*/
class OMUCircle{
public:
    __device__ static inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)
    {
        if ( f1 <= RANGE & f2 <= RANGE ){
            Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_L );
            if (id < 0)
                return 0.f;
            return Operator3D_cu(id).f(f1, f2, gf1, gf2);
        } else {
    #ifdef HACK
            Vec3_cu gf1n = gf1.normalized();
            Vec3_cu gf2n = gf2.normalized();
            float dot = gf1n.dot(gf2n);
            float tan_alpha = Blending_env::global_controller_fetch(dot).x;
            float tan_t = (f1 < f2) ? (f1/f2) : (f2/f1);

            if(tan_t  > tan_alpha)
            {
                float r  = sqrtf(f1 * f1 + f2 * f2);

                float res0 = Blending_env::arc_fetch(X0, X0*tan_t, tan_alpha)*0.5f;
                float r0   = sqrtf(X0*X0 + X0*X0*tan_t*tan_t);
                return res0 * r / r0;
            }else
                return fmax(f1, f2);
    #else
            return 2.f;
    #endif // HACK
        }
    }

    __device__ static inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)
    {
        if ( f1 <= RANGE & f2 <= RANGE ){
            Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_L );
            if (id < 0)
                return Vec3_cu();
            return Operator3D_cu(id).gf(f1, f2, gf1, gf2);
        } else {
    #ifdef HACK
            Vec3_cu gf1n = gf1.normalized();
            Vec3_cu gf2n = gf2.normalized();
            float dot = gf1n.dot(gf2n);
            float tan_alpha = Blending_env::global_controller_fetch(dot).x;
            float tan_t = (f1 < f2) ? (f1/f2) : (f2/f1);

            if(tan_t  > tan_alpha)
            {
                float r     = sqrtf(f1 * f1 + f2 * f2);

                float2 dg0  = Blending_env::arc_gradient_fetch(X0, X0*tan_t, tan_alpha);
                float  r0   = sqrtf(X0*X0 + X0*X0*tan_t*tan_t);
                float  fact = r / r0;
                dg0.x *=  fact;
                dg0.y *=  fact;

                if(f1 < f2) return gf1*dg0.y + gf2*dg0.x;
                else        return gf1*dg0.x + gf2*dg0.y;
            }else
                return f1 > f2 ? gf1 : gf2;
    #endif
            return Vec3_cu();
        }
    }

    __device__ static inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)
    {
        if ( (f1 <= RANGE) & (f2 <= RANGE) ){
            Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_L );
            if (id < 0){
                gf = Vec3_cu();
                return 0.f;
            }
            return Operator3D_cu(id).fngf(f1, f2, gf1, gf2, gf);
        } else {
    #ifdef HACK
            Vec3_cu gf1n = gf1.normalized();
            Vec3_cu gf2n = gf2.normalized();
            float dot = gf1n.dot(gf2n);
            float tan_alpha = Blending_env::global_controller_fetch(dot).x;
            float tan_t = (f1 < f2) ? (f1/f2) : (f2/f1);

            if(tan_t  > tan_alpha)
            {
                float r  = sqrtf(f1 * f1 + f2 * f2);

                float res0 = Blending_env::arc_fetch(X0, X0*tan_t, tan_alpha)*0.5f;
                float r0   = sqrtf(X0*X0 + X0*X0*tan_t*tan_t);

                float2 dg0  = Blending_env::arc_gradient_fetch(X0, X0*tan_t, tan_alpha);

                float  fact = r / r0;
                dg0.x *=  fact;
                dg0.y *=  fact;

                if(f1 < f2) gf = gf1*dg0.y + gf2*dg0.x;
                else        gf = gf1*dg0.x + gf2*dg0.y;

                return (res0 * fact);
            }
            else
            {
                gf = f1 > f2 ? gf1 : gf2;
                return fmax(f1, f2);
            }
    #else
            gf = Vec3_cu();
            return  2.f;
    #endif
        }
    }
};

/**
 * @class Op_tex
 * @brief base class for binnary operators defined <b>entirely</b> in texture
 * memory
 *
 * One has to provide the correct texture fetching functions through the
 * template parameter 'Fetch'
 *
 * needed functions :
 * @code
 * __device__ static inline
 * float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha);
 *
 * __device__ static inline
 * Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha);
 *
 * __device__ static inline
 * float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf);
 * @endcode
*/
template<class Fetch>
class Op_tex {
public:
    __device__ static inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)
    {
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;

        if( f1 < 1.f & f2 < 1.f )
            return Fetch::tex_f(f1, f2, gf1n, gf2n, tan_alpha);
        else
            return fmaxf(f1, f2);
    }

    __device__ static inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)
    {
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;

        if( f1 < 1.f & f2 < 1.f ){
            return Fetch::tex_gf(f1, f2, gf1n, gf2n, tan_alpha);
        } else {
            if( f1 < f2 ) return gf2;
            else          return gf1;
        }
    }

    __device__ static inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)
    {
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;

        if( (f1 < 1.0f) | (f2 < 1.f) )
            return Fetch::tex_fngf(f1, f2, gf1n, gf2n, tan_alpha, gf);
        else
        {
            if( f1 < f2 ){ gf = gf2; return f2; }
            else         { gf = gf1; return f1; }
        }
    }
};

// -----------------------------------------------------------------------------

/** @class Circle_anim
  @brief Blending operator dedicated for the implicit skinning (knee and elbow)
*/
class Circle_anim : public Op_tex<Circle_anim>  {
public:

    __device__ static inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
        if (id < 0)
            return 0.f;
        return Operator3D_cu(id).f(f1, f2, gf1, gf2);
    }

    __device__ static inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
        if (id < 0)
            return Vec3_cu();
        return Operator3D_cu(id).gf(f1, f2, gf1, gf2);
    }

    __device__ static inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
        if (id < 0){
            gf = Vec3_cu();
            return 0.f;
        }
        return Operator3D_cu(id).fngf(f1, f2, gf1, gf2, gf);
    }
};

// -----------------------------------------------------------------------------


/** @class Circle_HyperbolaOpen
  @brief Blending operator using Circle profile and open hyperbola
*/
// keep for blob_evaluator.hpp; TODO : delete completely
class Circle_HyperbolaOpen : public Op_tex<Circle_HyperbolaOpen> {
public:

    __device__ static inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_OH );
        if (id < 0)
            return 0.f;
        return Operator3D_cu(id).f(f1, f2, gf1, gf2);
    }

    __device__ static inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_OH );
        if (id < 0)
            return Vec3_cu();
        return Operator3D_cu(id).gf(f1, f2, gf1, gf2);
    }

    __device__ static inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_OH );
        if (id < 0){
            gf = Vec3_cu();
            return 0.f;
        }
        return Operator3D_cu(id).fngf(f1, f2, gf1, gf2, gf);
    }
};

// -----------------------------------------------------------------------------

/** @class Circle_HyperbolaClosed
  @brief Blending operator using Circle profile and closed hyperbola
  @param CLOSE_TYPE Defines closing function : @value -1 : diamond-like
                                               @value 0 : hermite-like
                                               @value 1 : tanh-like
*/
// keep for blob_evaluator.hpp; TODO : delete completely
template<int CLOSE_TYPE>
class Circle_HyperbolaClosed : public Op_tex<Circle_HyperbolaClosed<CLOSE_TYPE> > {
public:

    __device__ static inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case -1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
            break;
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_TCH );
            break;
        }
        if (id < 0)
            return 0.f;
        return Operator3D_cu(id).f(f1, f2, gf1, gf2);
    }

    __device__ static inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case -1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
            break;
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_TCH );
            break;
        }
        if (id < 0)
            return Vec3_cu();
        return Operator3D_cu(id).gf(f1, f2, gf1, gf2);
    }

    __device__ static inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case -1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
            break;
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_TCH );
            break;
        }
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Operator3D_cu(id).fngf(f1, f2, gf1, gf2, gf);
    }

};

// -----------------------------------------------------------------------------

/** @class Ultimate_HyperbolaClosed
  @brief Blending operator using Ultimate profile and colsed hyperbola
  @param CLOSE_TYPE Defines closing function : @value 0 : hermite-like
                                               @value 1 : tanh-like
*/
// keep for blob_evaluator.hpp; TODO : delete completely
template<int CLOSE_TYPE>
class Ultimate_HyperbolaClosed : public Op_tex<Ultimate_HyperbolaClosed<CLOSE_TYPE> > {
public:

    __device__ static inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_TCH );
            break;
        }
        if (id < 0)
            return 0.f;
        return Operator3D_cu(id).f(f1, f2, gf1, gf2);
    }

    __device__ static inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_TCH );
            break;
        }
        if (id < 0)
            return Vec3_cu();
        return Operator3D_cu(id).gf(f1, f2, gf1, gf2);
    }

    __device__ static inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_TCH );
            break;
        }
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Operator3D_cu(id).fngf(f1, f2, gf1, gf2, gf);
    }
};

// -----------------------------------------------------------------------------

/** @class Bulge_HyperbolaClosed
  @brief Bulge operator using bulge profile and closed hyperbola
  @param CLOSE_TYPE Defines closing function : @value 0 : hermite-like
                                               @value 1 : tanh-like
*/
// keep for blob_evaluator.hpp; TODO : delete completely
template<int CLOSE_TYPE>
class Bulge_HyperbolaClosed : public Op_tex<Bulge_HyperbolaClosed<CLOSE_TYPE> > {
public:

    __device__ static inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_TCH );
            break;
        }
        if (id < 0)
            return 0.f;
        return Operator3D_cu(id).f(f1, f2, gf1, gf2);
    }

    __device__ static inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_TCH );
            break;
        }
        if (id < 0)
            return Vec3_cu();
        return Operator3D_cu(id).gf(f1, f2, gf1, gf2);
    }

    __device__ static inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf)
    {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE) {
        case 0:
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_HCH );
            break;
        case 1:
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_TCH );
            break;
        }
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Operator3D_cu(id).fngf(f1, f2, gf1, gf2, gf);
    }


};

// -----------------------------------------------------------------------------

/** @class Ultimate_HyperbolaRestricted
  @brief Blending operator using Ultimate profile and hyperbola with restricted opening
  @param type Defines restricting function : @value 0 : truncate
                                             @value 1 : potential variation
*/
template<int TYPE>
class Ultimate_HyperbolaRestricted: public Op_tex< Ultimate_HyperbolaRestricted<TYPE> > {
public:

    __device__ static inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::U_OH );
        if (id < 0)
            return 0.f;
        float result = BulgeFreeBlending::f(f1, f2, gf1, gf2);

        if (f1>0.5 && f2>0.5){
            if (result > 1.f)
                return 1.f;
            if (TYPE == 0)
                return result;
            else
                return 0.f;
        }
        if (f1>0.5){
            return f1;
        }
        if (f2>0.5){
            return f2;
        }
        return result;
    }

    __device__ static inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::U_OH );
        if (id < 0)
            return Vec3_cu();
        Vec3_cu gf;
        float result = BulgeFreeBlending::fngf(gf, f1, f2, gf1, gf2);

        if (f1>0.5 && f2>0.5 && result > 1.f){
            return Vec3_cu();
        }
        if (f1>0.5){
            return gf1;
        }
        if (f2>0.5){
            return gf2;
        }
        return gf;
    }

    __device__ static inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf)
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::U_OH );
        if (id < 0){
            gf = Vec3_cu();
            return 0.f;
        }
        float result = BulgeFreeBlending::fngf(gf, f1, f2, gf1, gf2);

        if (f1>tan_alpha/2 && f2>tan_alpha/2)
        {
            if (TYPE == 0) {
                if (result > 1.f){
                    gf = Vec3_cu();
                    return 1.f;
                }
            }
            else
            {
                gf = Vec3_cu();
                return 1.f;
            }
        }

        return result;
    }
};

struct URicci_4D : public Op_tex<URicci_4D> {
    __device__ static  inline
    float tex_f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha){
        if(f1 <= 0.5f & f2 <= 0.5f)
            return Blending_env::openable_ricci_4D_fetch(f1, f2, tan_alpha, Blending_env::n_3D_ricci_fetch());

        float off = (tan_alpha*0.5f);
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float r = sqrtf(dx*dx + dy*dy);

            float tant = (dx < dy) ? dx/dy : dy/dx;
            r /= Blending_env::profile_ricci_4D_fetch(tant, Blending_env::n_3D_ricci_fetch());

            return r + off;
        }

        return fmaxf(f1, f2);
    }

    __device__ static  inline
    Vec3_cu tex_gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha){
        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            float2 dg = Blending_env::openable_ricci_4D_gradient_fetch(f1,f2,tan_alpha,Blending_env::n_3D_ricci_fetch());
            return gf1*dg.x + gf2*dg.y;
        }

        float off = (tan_alpha*0.5f);
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float tan_t = (dx<dy) ? dx/dy : dy/dx;

            float2 dg;
            dg = Blending_env::profile_ricci_4D_normal_fetch(tan_t, Blending_env::n_3D_ricci_fetch());

            float df1, df2;
            if(dx < dy){ df1 = dg.x; df2 = dg.y; }
            else       { df1 = dg.y; df2 = dg.x; }

            return gf1*df1 + gf2*df2;
        }
        return (f1<f2) ? gf2 : gf1;
    }

    __device__ static  inline
    float tex_fngf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2, float tan_alpha, Vec3_cu& gf){
        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            float2 dg = Blending_env::openable_ricci_4D_gradient_fetch(f1,f2,tan_alpha, Blending_env::n_3D_ricci_fetch());
            gf = gf1*dg.x + gf2*dg.y;
            return Blending_env::openable_ricci_4D_fetch(f1,f2,tan_alpha, Blending_env::n_3D_ricci_fetch());
        }

        float off = tan_alpha * 0.5f;
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float r  = sqrtf(dx*dx + dy*dy);

            float tan_t = (dx < dy) ? dx/dy : dy/dx;
            float2 dg;

            r /= Blending_env::profile_ricci_4D_fetch(tan_t, Blending_env::n_3D_ricci_fetch());
            dg = Blending_env::profile_ricci_4D_normal_fetch(tan_t, Blending_env::n_3D_ricci_fetch());

            float df1, df2;
            if(dx < dy){ df1 = dg.x; df2 = dg.y; }
            else       { df1 = dg.y; df2 = dg.x; }

            gf =  gf1*df1 + gf2*df2;
            return r + off;
        }

        if(f1 < f2){
            gf = gf2;
            return f2;
        } else {
            gf = gf1;
            return f1;
        }
    }
};


#endif //UCLEAN_HPP_

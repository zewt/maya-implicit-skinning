/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#ifndef DYN_OPERATORS_HPP__
#define DYN_OPERATORS_HPP__

#include "ultimate.hpp"
#include "uclean.hpp"

/**
  @file dyn_operators.hpp

  In this file you 'll find blending operators which can behave differently
  depending on some parameters specific to the instance of the operator.
  So unlike other blending ops with static functions you must instantiate
  the operator with specific parameters. For instance class Dyn_circle_anim
  takes the identifier ctrl_id which defines a specific gradient controler.
  Such identifier is given in 'blending_env.hpp' when creating an instance
  of a controler

  @see blending_env.hpp
*/

// =============================================================================

/** @class Circle_anim
  @brief Blending operator dedicated for the implicit skinning (knee and elbow)
*/
class Dyn_circle_anim{
public:

    __device__ __host__ inline
    Dyn_circle_anim(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
        if (id < 0)
            return 0.f;
        return Dyn_Operator3D_cu(id, _ctrl_id).f(f1, f2, gf1, gf2);
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)  const
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
        if (id < 0)
            return Vec3_cu();
        return Dyn_Operator3D_cu(id, _ctrl_id).gf(f1, f2, gf1, gf2);
    }

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_D );
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Dyn_Operator3D_cu(id, _ctrl_id).fngf(f1, f2, gf1, gf2, gf);
    }

private:
    int _ctrl_id; ///< Controller id in the Blending_env namespace
};

// =============================================================================

/** @class Dyn_4D_bulge
  @brief Bulge in contact gradient controlled with a user defined magnitude

  This 2D bulge operator depend on the gradient angle and the magnitude of the
  bulge hence the 2D +2D == 4D .

  f = bulge(f0, f1, theta(g0, g1), magnitude)
*/
class Dyn_4D_bulge{
public:

    __device__ __host__ inline
    Dyn_4D_bulge(int ctrl_id, float mag) : _ctrl_id(ctrl_id), _magnitude(mag) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)  const
    {
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
        float result = Base_4D_bulge::f(f1, f2,tan_alpha, _magnitude);
        return result;
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)  const
    {
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
        float2 gd = Base_4D_bulge::gf(f1, f2, tan_alpha, _magnitude);
        return gf1 * gd.x + gf2 * gd.y;
    }


    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2)  const
    {
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
        float2 gd;
        float result = Base_4D_bulge::fngf(gd, f1, f2, tan_alpha, _magnitude);
        gf = gf1 * gd.x + gf2 * gd.y;
        return result;
    }

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
    float _magnitude; ///< strength of the bulge in contact
};

// =============================================================================


/** @class Dyn_OMUCircle
  @brief Blending operator dedicated for the implicit skinning using
         Circle profile and line opening
*/
class Dyn_OMUCircle{
public:

    __device__ __host__ inline
    Dyn_OMUCircle(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

__device__
float Dyn_OMUCircle::f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_L );
    if (id < 0)
        return 0.f;
    return Dyn_Operator3D_cu(id, _ctrl_id).f(f1, f2, gf1, gf2);
}

__device__
Vec3_cu Dyn_OMUCircle::gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_L );
    if (id < 0)
        return Vec3_cu();
    return Dyn_Operator3D_cu(id, _ctrl_id).gf(f1, f2, gf1, gf2);
}

__device__
float Dyn_OMUCircle::fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_L );
    if (id < 0) {
        gf = Vec3_cu();
        return 0.f;
    }
    return Dyn_Operator3D_cu(id, _ctrl_id).fngf(f1, f2, gf1, gf2, gf);
}

// =============================================================================


/** @class Dyn_Circle_Open_Hyperbola
  @brief Blending operator dedicated for the implicit skinning using
         Circle profile and open hyperbola
*/
class Dyn_Circle_Open_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Circle_Open_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

__device__
float Dyn_Circle_Open_Hyperbola::f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_OH );
    if (id < 0)
        return 0.f;
    return Dyn_Operator3D_cu(id, _ctrl_id).f(f1, f2, gf1, gf2);
}

__device__
Vec3_cu Dyn_Circle_Open_Hyperbola::gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_OH );
    if (id < 0)
        return Vec3_cu();
    return Dyn_Operator3D_cu(id, _ctrl_id).gf(f1, f2, gf1, gf2);
}

__device__
float Dyn_Circle_Open_Hyperbola::fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Blending_env::Op_id id = Blending_env::predefined_op_id_fetch( Blending_env::C_OH );
    if (id < 0){
        gf = Vec3_cu();
        return 0.f;
    }
    return Dyn_Operator3D_cu(id, _ctrl_id).fngf(f1, f2, gf1, gf2, gf);
}

// =============================================================================

/** @class Dyn_Ricci_4D
  @brief Blending operator dedicated for the implicit skinning using
         Ricci profile with parameterizable N and open hyperbola
*/
class Dyn_Ricci_4D{
public:

    __device__ __host__ inline
    Dyn_Ricci_4D(int ctrl_id, float N) : _ctrl_id(ctrl_id), _N(N) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2);

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
    float _N;         ///< N parameter for Ricci equation
};

__device__
float Dyn_Ricci_4D::f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Vec3_cu gf1n = gf1.normalized();
    Vec3_cu gf2n = gf2.normalized();
    float dot = gf1n.dot(gf2n);
    float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

    if(f1 <= 0.5f & f2 <= 0.5f)
        return Blending_env::openable_ricci_4D_fetch(f1, f2, tan_alpha, _N);

    float off = (tan_alpha*0.5f);
    if((f1 > off) & (f2 > off))
    {
        float dx = f1 - off;
        float dy = f2 - off;
        float r = sqrtf(dx*dx + dy*dy);

        float tant = (dx < dy) ? dx/dy : dy/dx;
        r /= Blending_env::profile_ricci_4D_fetch(tant, _N);

        return r + off;
    }

    return fmaxf(f1, f2);
}

__device__
Vec3_cu Dyn_Ricci_4D::gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Vec3_cu gf1n = gf1.normalized();
    Vec3_cu gf2n = gf2.normalized();
    float dot = gf1n.dot(gf2n);
    float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

    if(f1 <= 0.5f & f2 <= 0.5f)
    {
        float2 dg = Blending_env::openable_ricci_4D_gradient_fetch(f1,f2,tan_alpha,_N);
        return gf1*dg.x + gf2*dg.y;
    }

    float off = (tan_alpha*0.5f);
    if((f1 > off) & (f2 > off))
    {
        float dx = f1 - off;
        float dy = f2 - off;
        float tan_t = (dx<dy) ? dx/dy : dy/dx;

        float2 dg;
        dg = Blending_env::profile_ricci_4D_normal_fetch(tan_t, _N);

        float df1, df2;
        if(dx < dy){ df1 = dg.x; df2 = dg.y; }
        else       { df1 = dg.y; df2 = dg.x; }

        return gf1*df1 + gf2*df2;
    }
    return (f1<f2) ? gf2 : gf1;
}

__device__
float Dyn_Ricci_4D::fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
    Vec3_cu gf1n = gf1.normalized();
    Vec3_cu gf2n = gf2.normalized();
    float dot = gf1n.dot(gf2n);
    float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
    float2 dg;

    if(f1 <= 0.5f & f2 <= 0.5f)
    {
        dg = Blending_env::openable_ricci_4D_gradient_fetch(f1, f2, tan_alpha, _N);
        gf = (gf1 * dg.x + gf2 * dg.y);
        return Blending_env::openable_ricci_4D_fetch(f1, f2, tan_alpha, _N);
    }

    float off = tan_alpha * 0.5f;
    if((f1 > off) & (f2 > off))
    {
        float dx = f1 - off;
        float dy = f2 - off;
        float r  = sqrtf(dx*dx + dy*dy);

        float tan_t = (dx < dy) ? dx/dy : dy/dx;
        float2 dg;

        r /= Blending_env::profile_ricci_4D_fetch(tan_t, _N);
        dg = Blending_env::profile_ricci_4D_normal_fetch(tan_t, _N);

        float df1, df2;
        if(dx < dy){ df1 = dg.x; df2 = dg.y; }
        else       { df1 = dg.y; df2 = dg.x; }

        gf =  gf1*df1 + gf2*df2;
        return r + off;
    }

    if( f1 < f2 ){ gf = gf2; return f2; }
    else         { gf = gf1; return f1; }
}

// =============================================================================

/** @class Dyn_Ultimate_Open_Hyperbola
  @brief Blending operator dedicated for the implicit skinning using
         Ultimate profile and open hyperbola
*/
class Dyn_Ultimate_Open_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Ultimate_Open_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const;

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const;

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const;

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

__device__ inline
float Dyn_Ultimate_Open_Hyperbola::f(float f1,
                                     float f2,
                                     const Vec3_cu& gf1,
                                     const Vec3_cu& gf2) const
{
    Vec3_cu gf1n = gf1.normalized();
    Vec3_cu gf2n = gf2.normalized();
    float dot = gf1n.dot(gf2n);
    float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

    if( f1 < 1.f & f2 < 1.f ){
        return OCU<E_OCU::BLEND>::f(f1, f2, tan_alpha);
    } else
        return fmaxf(f1, f2);
}

__device__ inline
Vec3_cu Dyn_Ultimate_Open_Hyperbola::gf(float f1,
                                        float f2,
                                        const Vec3_cu& gf1,
                                        const Vec3_cu& gf2) const
{
    float2 dg;
    Vec3_cu gf1n = gf1.normalized();
    Vec3_cu gf2n = gf2.normalized();
    float dot = gf1n.dot(gf2n);
    float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

    if( f1 < 1.f & f2 < 1.f ){
        dg = OCU<E_OCU::BLEND>::gf(f1, f2, tan_alpha);
        return gf1*dg.x + gf2*dg.y;
    } else {
        if( f1 < f2 ) return gf2;
        else          return gf1;
    }
}

__device__ inline
float Dyn_Ultimate_Open_Hyperbola::fngf(Vec3_cu& gf,
                                        float f1,
                                        float f2,
                                        const Vec3_cu& gf1,
                                        const Vec3_cu& gf2) const
{
    float2 dg;
    Vec3_cu gf1n = gf1.normalized();
    Vec3_cu gf2n = gf2.normalized();
    float dot = gf1n.dot(gf2n);
    float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
    if( (f1 < 1.0f) | (f2 < 1.f) )
    {
        dg = OCU<E_OCU::BLEND>::gf(f1, f2, tan_alpha);
        gf = (gf1 * dg.x + gf2 * dg.y);
        return OCU<E_OCU::BLEND>::f(f1, f2, tan_alpha);
    }
    else
    {
        if( f1 < f2 ){ gf = gf2; return f2; }
        else         { gf = gf1; return f1; }
    }
}

// =============================================================================


/** @class Dyn_Circle_Closed_Hyperbola
  @brief Blending operator dedicated for the implicit skinning using
         Circle profile and closed hyperbola
  @param CLOSE_TYPE Defines closing function : @value 0 : hermite-like
                                               @value 1 : tanh-like
*/
template< int CLOSE_TYPE >
class Dyn_Circle_Closed_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Circle_Closed_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::C_TCH );
            break;
        }
        if (id < 0)
            return 0.f;
        return Dyn_Operator3D_cu(id, _ctrl_id).f(f1, f2, gf1, gf2);
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::C_TCH );
            break;
        }
        if (id < 0)
            return Vec3_cu();
        return Dyn_Operator3D_cu(id, _ctrl_id).gf(f1, f2, gf1, gf2);
    }

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::C_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::C_TCH );
            break;
        }
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Dyn_Operator3D_cu(id, _ctrl_id).fngf(f1, f2, gf1, gf2, gf);
    }

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

// =============================================================================

/** @class Dyn_Ultimate_Closed_Hyperbola
  @brief Blending operator dedicated for the implicit skinning using
         Ultimate profile and closed hyperbola
  @param CLOSE_TYPE Defines closing function : @value 0 : hermite-like
                                               @value 1 : tanh-like
*/
template< int CLOSE_TYPE>
class Dyn_Ultimate_Closed_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Ultimate_Closed_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::U_TCH );
            break;
        }
        if (id < 0)
            return 0.f;
        return Dyn_Operator3D_cu(id, _ctrl_id).f(f1, f2, gf1, gf2);
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::U_TCH );
            break;
        }
        if (id < 0)
            return Vec3_cu();
        return Dyn_Operator3D_cu(id, _ctrl_id).gf(f1, f2, gf1, gf2);
    }

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::U_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::U_TCH );
            break;
        }
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Dyn_Operator3D_cu(id, _ctrl_id).fngf(f1, f2, gf1, gf2, gf);
    }

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

// =============================================================================

/** @class Dyn_Bulge_Open_Hyperbola
  @brief Bulge operator dedicated for the implicit skinning
         using Bulge profile and open hyperbola
*/
class Dyn_Bulge_Open_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Bulge_Open_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__ inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

        if( f1 < 1.f & f2 < 1.f )
            return OCU<E_OCU::BULGE>::f(f1, f2, tan_alpha);
        else
            return fmaxf(f1, f2);
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        float2 dg;
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

        if( f1 < 1.f & f2 < 1.f ){
            dg = OCU<E_OCU::BULGE>::gf(f1, f2, tan_alpha);
            return gf1*dg.x + gf2*dg.y;
        } else {
            if( f1 < f2 ) return gf2;
            else          return gf1;
        }
    }

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
        float2 dg;
        if( (f1 < 1.0f) | (f2 < 1.f) )
        {
            dg = OCU<E_OCU::BULGE>::gf(f1, f2, tan_alpha);
            gf = (gf1 * dg.x + gf2 * dg.y);
            return OCU<E_OCU::BULGE>::f(f1, f2, tan_alpha);
        }
        else
        {
            if( f1 < f2 ){ gf = gf2; return f2; }
            else         { gf = gf1; return f1; }
        }
    }

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

// =============================================================================

/** @class Dyn_Bulge_Closed_Hyperbola
  @brief Bulge operator dedicated for the implicit skinning
         using Bulge profile and closed hyperbola
  @param CLOSE_TYPE Defines closing function : @value 0 : hermite-like
                                               @value 1 : tanh-like
*/
template< int CLOSE_TYPE >
class Dyn_Bulge_Closed_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Bulge_Closed_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::B_TCH );
            break;
        }
        if (id < 0)
            return 0.f;
        return Dyn_Operator3D_cu(id, _ctrl_id).f(f1, f2, gf1, gf2);
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::B_TCH );
            break;
        }
        if (id < 0)
            return Vec3_cu();
        return Dyn_Operator3D_cu(id, _ctrl_id).gf(f1, f2, gf1, gf2);
    }

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2) const {
        Blending_env::Op_id id = -1;
        switch (CLOSE_TYPE){
        case 0 :
            id = Blending_env::predefined_op_id_fetch( Blending_env::B_HCH );
            break;
        case 1 :
            id =  Blending_env::predefined_op_id_fetch( Blending_env::B_TCH );
            break;
        }
        if (id < 0) {
            gf = Vec3_cu();
            return 0.f;
        }
        return Dyn_Operator3D_cu(id, _ctrl_id).fngf(f1, f2, gf1, gf2, gf);
    }

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

// =============================================================================

/** @class Dyn_Ultimate_Restricted_Hyperbola
  @brief Blending operator dedicated for the implicit skinning using
         Ultimate profile and hyperbola with restricted opening
*/
class Dyn_Ultimate_Restricted_Hyperbola{
public:

    __device__ __host__ inline
    Dyn_Ultimate_Restricted_Hyperbola(int ctrl_id) : _ctrl_id(ctrl_id) {}

    __device__
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;


        if( f1 < 1.f & f2 < 1.f )
            return Ultimate_HyperbolaRestricted<1>::tex_f(f1, f2, gf1, gf2, tan_alpha);
        else
            return fmaxf(f1, f2);
    }

    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;

        if( f1 < 1.f & f2 < 1.f ){
            return Ultimate_HyperbolaRestricted<1>::tex_gf(f1, f2, gf1, gf2, tan_alpha);
        } else {
            if( f1 < f2 ) return gf2;
            else          return gf1;
        }
    }

    __device__ inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::controller_fetch(_ctrl_id, dot).x;
        if( (f1 < 1.0f) | (f2 < 1.f) )
        {
            return Ultimate_HyperbolaRestricted<1>::tex_fngf(f1, f2, gf1, gf2, tan_alpha, gf);
        }
        else
        {
            if( f1 < f2 ){ gf = gf2; return f2; }
            else         { gf = gf1; return f1; }
        }
    }

private:
    int   _ctrl_id;   ///< Controller id in the Blending_env namespace
};

#endif //DYN_OPERATORS_HPP__

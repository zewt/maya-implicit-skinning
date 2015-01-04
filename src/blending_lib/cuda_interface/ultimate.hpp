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
#ifndef ULTIMATE_HPP_
#define ULTIMATE_HPP_

/** @file ultimate.hpp
 *
 * @warning the following operators using cuda textures that must be bound
 * before being used
 * @see Blending_env
 */
#if !defined(BLENDING_ENV_HPP__)
#error "You must include blending_env.hpp before the inclusion of 'ultimate.hpp'"
#endif

/**
 * @namespace E_OCU
 * @brief holds the different types of blending for the OCU operator
 * @see OCU
*/
// =============================================================================
namespace E_OCU {
// =============================================================================

enum Union_t {
    BLEND = 0, ///< simple blend
    BULGE = 1  ///< Bulge profile
};

}// END E_OCU ==================================================================

template <E_OCU::Union_t type>
struct UltimateOperator;

/// Gradient based blend
typedef UltimateOperator<E_OCU::BLEND> BulgeFreeBlending;

/// Gradient based bulge in contact
typedef UltimateOperator<E_OCU::BULGE> BulgeInContact;

// =============================================================================

template <E_OCU::Union_t type>
struct OCU {

    __device__ static inline
    float f(float f1, float f2, float tan_alpha)
    {
        if(type == E_OCU::BLEND && Blending_env::predefined_op_id_fetch(Blending_env::U_OH) == -1){
            return 0.f;
        } else if (type == E_OCU::BULGE && Blending_env::predefined_op_id_fetch(Blending_env::B_OH) == -1){
            return 0.f;
        }

        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            if(type == E_OCU::BLEND) return Blending_env::openable_clean_union_fetch(f1,f2,tan_alpha);
            else                     return Blending_env::openable_clean_skin_fetch (f1,f2,tan_alpha);
        }

        float off = (tan_alpha*0.5f);
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float r = sqrtf(dx*dx + dy*dy);

            float tant = (dx < dy) ? dx/dy : dy/dx;
            if(type == E_OCU::BLEND) r /= Blending_env::hyperbola_fetch(tant);
            else                     r /= Blending_env::skin_fetch(tant);

            return r + off;
        }

        return fmaxf(f1, f2);
    }

    __device__ static inline
    float2 gf(float f1, float f2, float tan_alpha)
    {
        if(type == E_OCU::BLEND && Blending_env::predefined_op_id_fetch(Blending_env::U_OH) == -1){
            return make_float2(0.f, 0.f);
        } else if (type == E_OCU::BULGE && Blending_env::predefined_op_id_fetch(Blending_env::B_OH) == -1){
            return make_float2(0.f, 0.f);
        }

        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            if(type == E_OCU::BLEND) return Blending_env::openable_clean_union_gradient_fetch(f1,f2,tan_alpha);
            else                     return Blending_env::openable_clean_skin_gradient_fetch (f1,f2,tan_alpha);
        }

        float off = (tan_alpha*0.5f);
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float tan_t = (dx<dy) ? dx/dy : dy/dx;

            float2 dg;
            if(type == E_OCU::BLEND) dg = Blending_env::hyperbola_normal_fetch(tan_t);
            else                     dg = Blending_env::skin_normal_fetch(tan_t);

            float df1, df2;
            if(dx < dy){ df1 = dg.x; df2 = dg.y; }
            else       { df1 = dg.y; df2 = dg.x; }

            return make_float2(df1, df2);
        }

        return (f1<f2) ? make_float2(0.f,1.f) : make_float2(1.f,0.f);
    }


    __device__ static inline
    float fngf(float2& gf, float f1, float f2, float tan_alpha)
    {
        if(type == E_OCU::BLEND && Blending_env::predefined_op_id_fetch(Blending_env::U_OH) == -1){
            gf = make_float2(0.f, 0.f);
            return 0.f;
        } else if (type == E_OCU::BULGE && Blending_env::predefined_op_id_fetch(Blending_env::B_OH) == -1){
            gf = make_float2(0.f, 0.f);
            return 0.f;
        }

        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            if(type == E_OCU::BLEND){
                gf = Blending_env::openable_clean_union_gradient_fetch(f1,f2,tan_alpha);
                return Blending_env::openable_clean_union_fetch(f1,f2,tan_alpha);
            } else {
                gf = Blending_env::openable_clean_skin_gradient_fetch(f1,f2,tan_alpha);
                return Blending_env::openable_clean_skin_fetch(f1,f2,tan_alpha);
            }
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
            if(type == E_OCU::BLEND){
                r /= Blending_env::hyperbola_fetch(tan_t);
                dg = Blending_env::hyperbola_normal_fetch(tan_t);
            } else {
                r /= Blending_env::skin_fetch(tan_t);
                dg = Blending_env::skin_normal_fetch(tan_t);
            }

            float df1, df2;
            if(dx < dy){ df1 = dg.x; df2 = dg.y; }
            else       { df1 = dg.y; df2 = dg.x; }

            gf =  make_float2(df1, df2);
            return r + off;
        }

        if(f1 < f2){
            gf = make_float2(0.f,1.f);
            return f2;
        } else {
            gf = make_float2(1.f,0.f);
            return f1;
        }
    }
};

// END OCU =====================================================================

// =============================================================================
template <E_OCU::Union_t type>
struct UltimateOperator{

    __device__ static inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;
        float result = OCU<type>::f(f1, f2,tan_alpha);
        return result;
    }


    __device__ static inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;
        float2 gd = OCU<type>::gf(f1, f2, tan_alpha);
        return gf1 * gd.x + gf2 * gd.y;
    }

    __device__ static inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;
        float2 gd;
        float result = OCU<type>::fngf(gd, f1, f2, tan_alpha);
        gf = gf1 * gd.x + gf2 * gd.y;
        return result;
    }
};
// END UltimateOperator ========================================================

// =============================================================================

struct Base_4D_bulge{

    __device__ static inline
    float f(float f1, float f2, float tan_alpha, float strength)
    {
        if(f1 <= 0.5f & f2 <= 0.5f)
            return Blending_env::openable_bulge_4D_fetch(f1, f2, tan_alpha, strength);


        float off = (tan_alpha*0.5f);
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float r = sqrtf(dx*dx + dy*dy);

            float tant = (dx < dy) ? dx/dy : dy/dx;
            r /= Blending_env::profile_bulge_4D_fetch(tant, strength);

            return r + off;
        }

        return fmaxf(f1, f2);
    }

    __device__ static inline
    float2 gf(float f1, float f2, float tan_alpha, float strength)
    {
        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            return Blending_env::openable_bulge_4D_gradient_fetch(f1,f2,tan_alpha,strength);
        }

        float off = (tan_alpha*0.5f);
        bool cd = (f1 > off) & (f2 > off);
        if(cd)
        {
            float dx = f1 - off;
            float dy = f2 - off;
            float tan_t = (dx<dy) ? dx/dy : dy/dx;

            float2 dg;
            dg = Blending_env::profile_bulge_4D_normal_fetch(tan_t, strength);

            float df1, df2;
            if(dx < dy){ df1 = dg.x; df2 = dg.y; }
            else       { df1 = dg.y; df2 = dg.x; }

            return make_float2(df1, df2);
        }
        return (f1<f2) ? make_float2(0.f,1.f) : make_float2(1.f,0.f);
    }


    __device__ static inline
    float fngf(float2& gf, float f1, float f2, float tan_alpha, float strength)
    {
        if(f1 <= 0.5f & f2 <= 0.5f)
        {
            gf = Blending_env::openable_bulge_4D_gradient_fetch(f1,f2,tan_alpha, strength);
            return Blending_env::openable_bulge_4D_fetch(f1,f2,tan_alpha, strength);
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

            r /= Blending_env::profile_bulge_4D_fetch(tan_t, strength);
            dg = Blending_env::profile_bulge_4D_normal_fetch(tan_t, strength);

            float df1, df2;
            if(dx < dy){ df1 = dg.x; df2 = dg.y; }
            else       { df1 = dg.y; df2 = dg.x; }

            gf =  make_float2(df1, df2);
            return r + off;
        }

        if(f1 < f2){
            gf = make_float2(0.f,1.f);
            return f2;
        } else {
            gf = make_float2(1.f,0.f);
            return f1;
        }
    }
};

// =============================================================================

struct Static_4D_bulge{
    __device__ static inline
    float f(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;
        float result = Base_4D_bulge::f(f1, f2,tan_alpha, Blending_env::magnitude_3D_bulge_fetch());
        return result;
    }


    __device__ static inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;
        float2 gd = Base_4D_bulge::gf(f1, f2, tan_alpha, Blending_env::magnitude_3D_bulge_fetch());
        return gf1 * gd.x + gf2 * gd.y;
    }

    __device__ static inline
    float fngf(Vec3_cu& gf, float f1, float f2, const Vec3_cu& gf1, const Vec3_cu& gf2){
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float dot = gf1n.dot(gf2n);
        float tan_alpha = Blending_env::global_controller_fetch(dot).x;
        float2 gd;
        float result = Base_4D_bulge::fngf(gd, f1, f2, tan_alpha, Blending_env::magnitude_3D_bulge_fetch());
        gf = gf1 * gd.x + gf2 * gd.y;
        return result;
    }

};

// END STATIC_4D_BULGE =========================================================

// END UltimateOperator ========================================================



#endif // ULTIMATE_HPP_

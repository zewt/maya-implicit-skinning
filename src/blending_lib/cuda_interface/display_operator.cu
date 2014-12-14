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
#include "display_operator.hpp"

#include "blending_env.hpp"

#include "cuda_utils.hpp"

#include "blending_env_tex.hpp"

#include "uclean.hpp"
#include "ucommon.hpp"
#include "ultimate.hpp"
#include "n_ary.hpp"
#include "operator3d_cu.hpp"

#include "operators_ctrl.hpp"
namespace Cuda_ctrl{
extern Operators_ctrl _operators;
}

// -----------------------------------------------------------------------------

#include "opening.hpp"

/// specialized class to fetch data on device for Discret_hyperbola_base
/// to work in kernel code
class Pan_hf_cu {
public:
    /// @warning call from host is forbidden
    __device__ __host__
    float linear_fetch(float t) const{
        FORBID_HOST_CALL();
        #ifdef __CUDA_ARCH__
            return Blending_env::pan_hyperbola_fetch(t);
        #else
            return 0.f;
        #endif
    }
};


// -----------------------------------------------------------------------------
// TODO deal with custom operators (also in csg_eval)
__device__
float eval(Blending_env::Op_t type, Blending_env::Op_mode mode, Blending_env::Op_id custom_id, Vec3_cu& gf, float f0, float f1, const Vec3_cu& gf0, const Vec3_cu& gf1)
{
    switch(type)
    {
    case Blending_env::DIFF:
        return 1.f - Umax::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
    case Blending_env::DIFF_SMOOTH:
        return 1.f - Usum::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
    case Blending_env::MAX:
        return Umax::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::MIN:
        return Umin::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::SUM:
        return Usum::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::RICCI:
        return URicci::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::C_CONTACT:
        return CaniContact::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::C_CONTACT_UNAIRE:
        return CaniContactUnaire::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::W_RESTRICTED:
        return RestrictedBlend::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::W_RESTRICTED_UNAIRE:
        return RestrictedBlendUnaire::fngf(gf, f0, f1, gf0, gf1);
    case Blending_env::B_OH_4D:
        switch (mode) {
        case Blending_env::UNION:
            return Static_4D_bulge::fngf(gf, f0, f1, gf0, gf1);
        case Blending_env::INTERSECTION:
            return 1.f - Static_4D_bulge::fngf(gf, 1.f-f0, 1.f-f1, gf0, gf1);
        case Blending_env::DIFFERENCE:
            return 1.f - Static_4D_bulge::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
        }
    case Blending_env::R_OH_4D:
        switch (mode) {
        case Blending_env::UNION:
            return URicci_4D::fngf(gf, f0, f1, gf0, gf1);
        case Blending_env::INTERSECTION:
            return 1.f - URicci_4D::fngf(gf, 1.f-f0, 1.f-f1, gf0, gf1);
        case Blending_env::DIFFERENCE:
            return 1.f - URicci_4D::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
        }

    case Blending_env::C_D:
    case Blending_env::C_OH:
    case Blending_env::C_HCH:
    case Blending_env::C_TCH:
    case Blending_env::U_HCH:
    case Blending_env::U_TCH:
    case Blending_env::B_HCH:
    case Blending_env::B_TCH:
    case Blending_env::B_D:
    // for each 3D binary operator
    {
        Blending_env::Op_id id = Blending_env::predefined_op_id_fetch(type);
        if (id >= 0) {
            Operator3D_cu op(id);
            switch (mode) {
            case Blending_env::UNION:
                return op.fngf(f0, f1, gf0, gf1, gf);
            case Blending_env::INTERSECTION:
                return 1.f - op.fngf(1.f-f0, 1.f-f1, gf0, gf1, gf);
            case Blending_env::DIFFERENCE:
                return 1.f - op.fngf(1.f-f0, f1, gf0, gf1, gf);// don't revert gf1 for display
            }
        } else {
            gf = Vec3_cu();
            return 0.f;
        }
    }

    // special case because of line => can be > 1
    case Blending_env::C_L:
        switch (mode) {
            case Blending_env::UNION:
                return OMUCircle::fngf(gf, f0, f1, gf0, gf1) * 2.f;
            case Blending_env::INTERSECTION:
                return 1.f - OMUCircle::fngf(gf, 1.f-f0, 1.f-f1, gf0, gf1) * 2.f;
            case Blending_env::DIFFERENCE:
                return 1.f - OMUCircle::fngf(gf, 1.f-f0, f1, gf0, gf1) * 2.f;// don't revert gf1 for display
        }
    // special cases because of open hyperbola => can be > 1
    case Blending_env::U_OH:
        switch (mode) {
        case Blending_env::UNION:
            return BulgeFreeBlending::fngf(gf, f0, f1, gf0, gf1);
        case Blending_env::INTERSECTION:
            return 1.f - BulgeFreeBlending::fngf(gf, 1.f-f0, 1.f-f1, gf0, gf1);
        case Blending_env::DIFFERENCE:
            return 1.f - BulgeFreeBlending::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
        }
    case Blending_env::U_RH:
    {
        switch (mode) {
            case Blending_env::UNION:
                return Ultimate_HyperbolaRestricted<0>::fngf(gf, f0, f1, gf0, gf1);
            case Blending_env::INTERSECTION:
                return 1.f - Ultimate_HyperbolaRestricted<0>::fngf(gf, 1.f-f0, 1.f-f1, gf0, gf1);
            case Blending_env::DIFFERENCE:
                 return 1.f - Ultimate_HyperbolaRestricted<0>::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
        }
        gf = Vec3_cu();
        return 0.f;
    }
    case Blending_env::B_OH:
        switch (mode) {
            case Blending_env::UNION:
                return BulgeInContact::fngf(gf, f0, f1, gf0, gf1);
            case Blending_env::INTERSECTION:
                return 1.f - BulgeInContact::fngf(gf, 1.f-f0, 1.f-f1, gf0, gf1);
            case Blending_env::DIFFERENCE:
                return 1.f - BulgeInContact::fngf(gf, 1.f-f0, f1, gf0, gf1);// don't revert gf1 for display
        }

    case Blending_env::CUSTOM:
        if (custom_id >= 0) {
            Operator3D_cu op(custom_id);
            switch (mode) {
            case Blending_env::UNION:
                return op.fngf(f0, f1, gf0, gf1, gf);
            case Blending_env::INTERSECTION:
                return 1.f - op.fngf(1.f-f0, 1.f-f1, gf0, gf1, gf);
            case Blending_env::DIFFERENCE:
                return 1.f - op.fngf(1.f-f0, f1, gf0, gf1, gf);// don't revert gf1 for display
            }
        } else {
            gf = Vec3_cu();
            return 0.f;
        }
    default:
        gf = Vec3_cu(0.f, 0.f, 0.f);
        return 0.f;
    }
}

// -----------------------------------------------------------------------------

#define compute_dtan(opening, ddx, ddy, tant) \
    (ddx<ddy) ? ( (ddx) -  opening.f( (ddy) * 2.f, tant) * 0.5f ) : \
                ( (ddy) -  opening.f( (ddx) * 2.f, tant) * 0.5f )


/// @return distance to the opening line of the point ddx, ddy for an angle
/// tant of the opening
__device__
float eval_opening(Blending_env::Op_t type, Blending_env::Op_mode mode, Blending_env::Op_id custom_id, float ddx, float ddy, float tant)
{
    // TODO: dtan should be more precise (use opening slope perhaps ?)
    float dtan = 100.f;
    typedef IBL::Opening::Discreet_hyperbola_base<Pan_hf_cu> Dh_cu;
    Dh_cu closed_tanh(Dh_cu::CLOSED_TANH   );
    Dh_cu closed_hermite(Dh_cu::CLOSED_HERMITE);
    Dh_cu open_tanh(Dh_cu::OPEN_TANH);
    IBL::Opening::Diamond diamond;

    switch(type)
    {
    case Blending_env::C_L:{
        switch (mode) {
        case Blending_env::UNION:
            dtan = (ddx<ddy) ? (ddx -  ddy*tant) : (ddy -  ddx*tant);
            break;
        case Blending_env::INTERSECTION:
            ddx = 1.f - ddx;
            ddy = 1.f - ddy;
            dtan = (ddx<ddy) ? (ddx -  ddy*tant) : (ddy -  ddx*tant);
            break;
        case Blending_env::DIFFERENCE:
            ddx = 1.f - ddx;
            dtan = (ddx<ddy) ? (ddx -  ddy*tant) : (ddy -  ddx*tant);
            break;
        }
    }break;

    case Blending_env::C_D:
    case Blending_env::B_D: {
        switch (mode) {
        case Blending_env::UNION:
            dtan = compute_dtan(diamond, ddx, ddy, tant);
            break;
        case Blending_env::INTERSECTION:
            ddx = 1.f - ddx;
            ddy = 1.f - ddy;
            dtan = compute_dtan(diamond, ddx, ddy, tant);
            break;
        case Blending_env::DIFFERENCE:
            ddx = 1.f - ddx;
            dtan = compute_dtan(diamond, ddx, ddy, tant);
            break;
        }
    }break;

    case Blending_env::B_OH_4D:
    case Blending_env::R_OH_4D:
    case Blending_env::U_RH:
    case Blending_env::C_OH:
    case Blending_env::U_OH:
    case Blending_env::B_OH: {
        switch (mode) {
        case Blending_env::UNION:
            dtan = compute_dtan(open_tanh, ddx, ddy, tant);
            break;
        case Blending_env::INTERSECTION:
            ddx = 1.f - ddx;
            ddy = 1.f - ddy;
            dtan = compute_dtan(open_tanh, ddx, ddy, tant);
            break;
        case Blending_env::DIFFERENCE:
            ddx = 1.f - ddx;
            dtan = compute_dtan(open_tanh, ddx, ddy, tant);
            break;
        }
    }break;

    case Blending_env::C_HCH:
    case Blending_env::U_HCH:
    case Blending_env::B_HCH:{
        switch (mode) {
        case Blending_env::UNION:
            dtan = compute_dtan(closed_hermite, ddx, ddy, tant);
            break;
        case Blending_env::INTERSECTION:
            ddx = 1.f - ddx;
            ddy = 1.f - ddy;
            dtan = compute_dtan(closed_hermite, ddx, ddy, tant);
            break;
        case Blending_env::DIFFERENCE:
            ddx = 1.f - ddx;
            dtan = compute_dtan(closed_hermite, ddx, ddy, tant);
            break;
        }
    }break;

    case Blending_env::C_TCH:
    case Blending_env::U_TCH:
    case Blending_env::B_TCH:{
        switch (mode) {
        case Blending_env::UNION:
            dtan = compute_dtan(closed_tanh, ddx, ddy, tant);
            break;
        case Blending_env::INTERSECTION:
            ddx = 1.f - ddx;
            ddy = 1.f - ddy;
            dtan = compute_dtan(closed_tanh, ddx, ddy, tant);
            break;
        case Blending_env::DIFFERENCE:
            ddx = 1.f - ddx;
            dtan = compute_dtan(closed_tanh, ddx, ddy, tant);
            break;
        }
    }break;

    case Blending_env::CUSTOM:
    {
        // TODO : fetch texture of "boundary" per custom_op ?
    }

    default: { dtan = 100.f; }break;
    }
    return dtan;
}

// -----------------------------------------------------------------------------

__device__
float raise_power(float a){
    float g = a;
    g*= g;
    g*= g;
    g*= g;
    g*= g;
    g*= g;
    return g;
}

// -----------------------------------------------------------------------------

__global__
void draw_operator_kernel(float4* output_pbo,
                          int width, int height,
                          Blending_env::Op_t type,
                          Blending_env::Op_mode mode,
                          float angle,
                          Blending_env::Op_id custom_id)
{
    const int px = blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    const float line_min = 0.03f;

    if (px >= width || py >= height) return;

    const int p = py * width + px;
    float r, g, b;

    float ddx = px*1. / width;
    float ddy = py*1. / height;

    // Input gradient for which we draw the controler
    Vec3_cu gf0(1.f, 0.f, 0.f);
    Vec3_cu gf1(0.f, 1.f, 0.f);
    float tant = angle;

    Vec3_cu ggres;
    float ddres = eval(type, mode, custom_id, ggres, ddx, ddy, gf0, gf1);
    float2 gdres = make_float2(ggres.dot(gf0), ggres.dot(gf1));

    float dtan = eval_opening(type, mode, custom_id, ddx, ddy, tant);

    g = cosf(ddres*M_PI*9.f*2.f);
    g = raise_power(g);
    g*= g;
    r = gdres.x;
    b = gdres.y;
    if(fabsf(ddres-0.5f)<line_min)
    {
        r = fmaxf(g, r);
        b = fmaxf(g, b);
        float u = g;
        g = cosf( (fabsf(dtan) < line_min) ? fabsf(dtan) * 50.f : 1.f );
        g = raise_power(g);
        r -= g;
        b -= g;
        g = fmaxf(g,u);
    }
    else
    {
        r *= (1.f - g);
        b *= (1.f - g);
        g = (fabsf(dtan) < 0.003f) ? 1.f : 0.f;
        g = cosf( (fabsf(dtan) < line_min) ? fabsf(dtan) * 50.f : 1.f );
        g = raise_power(g);
        r -= g;
        b -= g;
    }

    r = fmaxf(0.f,fminf(1.f,r));
    g = fmaxf(0.f,fminf(1.f,g));
    b = fmaxf(0.f,fminf(1.f,b));
    //pbo[p] = int(r*255) + (int(g*255)<<8) + (int(b*255)<<16);
    output_pbo[p] = make_float4(r,g,b,1.f);
}

// -----------------------------------------------------------------------------

/// @return operator texture array in host memory
float* compute_tex_operator(int width, int height,
                            Blending_env::Op_t type,
                            Blending_env::Op_mode mode,
                            float angle,
                            Blending_env::Op_id custom_id = -1)
{
    Blending_env::bind_local();
    using namespace Cuda_utils;

    dim3 dblock(8, 8);
    dim3 dgrid((width + 8 - 1)/8, (height + 8 - 1)/8);

    float4* d_tex;
    malloc_d(d_tex ,width*height);

    float4* h_tex = new float4[width*height];

    IBL::Ctrl_setup save_shape = Cuda_ctrl::_operators.get_global_controller();

    angle = 1.f-angle;
    IBL::float2 p0 = {-3.f, angle};
    IBL::float2 p1 = {0.0f, angle};
    IBL::float2 p2 = {3.0f, angle};

    IBL::Ctrl_setup s(p0, p1, p2, 0.f, 0.f);
    Cuda_ctrl::_operators.set_global_controller(s);

    draw_operator_kernel<<<dgrid, dblock>>>(d_tex,
                                            width, height,
                                            type, mode,
                                            angle,
                                            custom_id);
    Blending_env::unbind_local();

    Cuda_ctrl::_operators.set_global_controller(save_shape);

    mem_cpy_dth(h_tex, d_tex, width*height);
    CUDA_SAFE_CALL(cudaFree(d_tex));
    return (float*)h_tex;
}

// -----------------------------------------------------------------------------

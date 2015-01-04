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
#pragma once
#include "hermiteRBF.hpp"
#include "distance_field.hpp"

#if !defined(HRBF_ENV_TEX_HPP__)
#error "You must include hrbf_env_tex.hpp before the inclusion of 'hermiteRBF.inl'"
#endif

IF_CUDA_DEVICE_HOST inline
float HermiteRBF::fngf_global(Vec3_cu& grad, const Point_cu& x) const
{
    grad = Vec3_cu(0., 0., 0.);

    float ret  = 0;
    int2 size_off = HRBF_env::fetch_inst_size_and_offset(_id);

    if(size_off.y == 0) return 0.f;

    for(int i = 0; i<size_off.y; i++)
    {
        Point_cu  node;
        Vec3_cu beta;
        float alpha     = HRBF_env::fetch_weights_point(beta, node, i+size_off.x);
        Vec3_cu diff  = x - node;

        Vec3_cu diffNormalized = diff;
        float l = diffNormalized.safe_normalize();

        // thin plates + generalisation
        #if defined(HERMITE_WITH_X3)
        float _3l      = 3 * l;
        float alpha3l  = alpha * _3l;
        float bDotd3   = beta.dot(diff) * 3;

        grad.x += alpha3l * diff.x;
        grad.x += beta.x * _3l + diffNormalized.x * bDotd3;

        grad.y += alpha3l * diff.y;
        grad.y += beta.y * _3l + diffNormalized.y * bDotd3;

        grad.z += alpha3l * diff.z;
        grad.z += beta.z * _3l + diffNormalized.z * bDotd3;

        ret += (alpha * l * l + beta.dot(diff) * 3.f) * l ;

        #elif defined(HERMITE_RBF_HPP__)
        // cf wxMaxima with function = alpha * phi(sqrt((cx-x)^2 + (cy-y)^2 + (cz-z)^2))
        //                             + dphi(sqrt((cx-x)^2 + (cy-y)^2 + (cz-z)^2)) * ((cx-x)*bx + (cy-y)*by + (cz-z)*bz) / sqrt((cx-x)^2 + (cy-y)^2 + (cz-z)^2);

        if( l > 0.00001f)
        {
            float dphi = RBFWrapper::PHI_TYPE::df(l);
            float ddphi = RBFWrapper::PHI_TYPE::ddf(l);

            float alpha_dphi = alpha * dphi;

            float bDotd_l = beta.dot(diff)/l;
            float squared_l = diff.norm_squared();

            grad.x += alpha_dphi * diffNormalized.x;
            grad.x += bDotd_l * (ddphi * diffNormalized.x - diff.x * dphi / squared_l) + beta.x * dphi / l ;

            grad.y += alpha_dphi * diffNormalized.y;
            grad.y += bDotd_l * (ddphi * diffNormalized.y - diff.y * dphi / squared_l) + beta.y * dphi / l ;

            grad.z += alpha_dphi * diffNormalized.z;
            grad.z += bDotd_l * (ddphi * diffNormalized.z - diff.z * dphi / squared_l) + beta.z * dphi / l ;

            ret += alpha * RBFWrapper::PHI_TYPE::f(l) + beta.dot(diff)*dphi/l;
        }
        #endif

    }

    return ret;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
float HermiteRBF::fngf(Vec3_cu& grad, const Point_cu& x) const
{
    const float ret = fngf_global(grad, x);
#if defined(POLY_C2)
    Field::grad_to_compact_poly_c2(ret, HRBF_env::fetch_radius(_id), grad);
    return Field::to_compact_poly_c2(ret, HRBF_env::fetch_radius(_id));
#elif defined(TANH_CINF)
    Field::grad_to_compact_tanh(ret, HRBF_env::fetch_radius(_id), TO_, grad);
    return Field::to_compact_tanh(ret, HRBF_env::fetch_radius(_id), TO_);
#else
    return ret;
#endif
}

// -----------------------------------------------------------------------------

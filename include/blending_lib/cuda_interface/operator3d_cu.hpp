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
#ifndef OPERATOR_HPP
#define OPERATOR_HPP

#include "cuda_compiler_interop.hpp"
#include "idx3_cu.hpp"
#include "vec3_cu.hpp"
#include "blending_env_tex.hpp"

/** @class Operator3D_cu
    @brief 3D CSG operators representation class
  */
struct Operator3D_cu {
    /// @param op_id : Operator index in @see Blending_env
    /// @warning op_id MUST result from a call to /// @see Blending_env::new_op_instance
    IF_CUDA_DEVICE_HOST
    Operator3D_cu(Blending_env::Op_id op_id) : _op_idx(op_id){
    }

    /// @param f1, f2 : values of composed implicit surfaces
    /// @param gf1, gf2 : gradients of composed implicit surfaces
    /// @returns the composition value of f1 and f2 by @see _op_idx operator
    /// with global controller
    __device__ inline
    float f(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2) const {
        Idx3_cu id = Blending_env::operator_idx_offset_fetch( _op_idx );
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float tan_alpha = Blending_env::global_controller_fetch( gf1n.dot(gf2n) ).x;
        return Blending_env::operator_fetch( id, f1, f2, tan_alpha );
    }

    /// @param f1, f2 : values of composed implicit surfaces
    /// @param gf1, gf2 : gradients of composed implicit surfaces
    /// @returns the composition gradient of f1 and f2 by @see _op_idx operator
    /// with global controller
    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2) const {
        Idx3_cu id = Blending_env::operator_idx_offset_fetch( _op_idx );
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float tan_alpha = Blending_env::global_controller_fetch( gf1n.dot(gf2n) ).x;
        float2 dg = Blending_env::operator_grad_fetch( id, f1, f2, tan_alpha );
        return gf1 *dg.x + gf2 * dg.y;
    }

    /// @param f1, f2 : values of composed implicit surfaces
    /// @param gf1, gf2 : gradients of composed implicit surfaces
    /// @param gf the composition gradient of f1 and f2 by @see _op_idx operator
    /// with global controller
    /// @returns the composition value of f1 and f2 by @see _op_idx operator
    /// with global controller
    __device__ inline
    float fngf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2, Vec3_cu &gf) const {
        Idx3_cu id = Blending_env::operator_idx_offset_fetch( _op_idx );
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float tan_alpha = Blending_env::global_controller_fetch( gf1n.dot(gf2n) ).x;
        float2 dg = Blending_env::operator_grad_fetch( id, f1, f2, tan_alpha );
        gf = gf1 *dg.x + gf2 * dg.y;
        return Blending_env::operator_fetch( id, f1, f2, tan_alpha );
    }

private:
    Blending_env::Op_id _op_idx; ///< Operator index in @see Blending_env
};

/** @class Dyn_Operator3D_cu
    @brief Dynamic 3D CSG operators representation class
  */
struct Dyn_Operator3D_cu {
    /// @param op_id : Operator index in @see Blending_env
    /// @param ctrl_id : Controller index in @see Blending_env
    /// @warning op_id MUST result from a call to /// @see Blending_env::new_op_instance
    /// @warning ctrl_id MUST result from a call to /// @see Blending_env::new_ctrl_instance
    IF_CUDA_DEVICE_HOST
    Dyn_Operator3D_cu(Blending_env::Op_id op_id, Blending_env::Ctrl_id ctrl_id) :
        _op_idx(op_id), _ctrl_idx(ctrl_id) {
    }

    /// @param f1, f2 : values of composed implicit surfaces
    /// @param gf1, gf2 : gradients of composed implicit surfaces
    /// @returns the composition value of f1 and f2 by @see _op_idx operator
    /// with @see _ctrl_idx controller
    __device__ inline
    float f(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2) const {
        Idx3_cu id = Blending_env::operator_idx_offset_fetch( _op_idx );
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float tan_alpha = Blending_env::controller_fetch( _ctrl_idx, gf1n.dot(gf2n) ).x;
        return Blending_env::operator_fetch(id, f1, f2, tan_alpha);
    }

    /// @param f1, f2 : values of composed implicit surfaces
    /// @param gf1, gf2 : gradients of composed implicit surfaces
    /// @returns the composition gradient of f1 and f2 by @see _op_idx operator
    /// with @see _ctrl_idx controller
    __device__ inline
    Vec3_cu gf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2) const {
        Idx3_cu id = Blending_env::operator_idx_offset_fetch( _op_idx );
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float tan_alpha = Blending_env::controller_fetch( _ctrl_idx, gf1n.dot(gf2n) ).x;
        float2 dg = Blending_env::operator_grad_fetch(id, f1, f2, tan_alpha);
        return gf1 *dg.x + gf2 * dg.y;
    }

    /// @param f1, f2 : values of composed implicit surfaces
    /// @param gf1, gf2 : gradients of composed implicit surfaces
    /// @param gf the composition gradient of f1 and f2 by @see _op_idx operator
    /// with @see _ctrl_idx controller
    /// @returns the composition value of f1 and f2 by @see _op_idx operator
    /// with @see _ctrl_idx controller
    __device__ inline
    float fngf(float f1, float f2, const Vec3_cu &gf1, const Vec3_cu &gf2, Vec3_cu &gf) const {
        Idx3_cu id = Blending_env::operator_idx_offset_fetch( _op_idx );
        Vec3_cu gf1n = gf1.normalized();
        Vec3_cu gf2n = gf2.normalized();
        float tan_alpha = Blending_env::controller_fetch( _ctrl_idx, gf1n.dot(gf2n) ).x;
        float2 dg = Blending_env::operator_grad_fetch(id, f1, f2, tan_alpha);
        gf = gf1 *dg.x + gf2 * dg.y;
        return Blending_env::operator_fetch(id, f1, f2, tan_alpha);
    }

private:
    Blending_env::Op_id _op_idx;     ///< Operator index in @see Blending_env
    Blending_env::Ctrl_id _ctrl_idx; ///< Controller index in @see Blending_env
};



#endif // OPERATOR_HPP

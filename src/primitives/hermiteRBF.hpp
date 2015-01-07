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
#ifndef HERMITE_RBF_HPP__
#define HERMITE_RBF_HPP__

#include <cassert>
#include <vector>
#include "cuda_compiler_interop.hpp"
#include "vec3_cu.hpp"
#include "point_cu.hpp"

// -----------------------------------------------------------------------------

//#define TANH_CINF
#define POLY_C2

#include "macros.hpp"
#define TO_     (7.)

// thin plates + generalisation
#include "hrbf_wrapper.hpp"

// -----------------------------------------------------------------------------


/** @brief Implicit primitive build over a set of normals and points

    The scalar field of this class is reconstructed from a set of points
    and normals with the Hermite RBF technique.

    Points and normals are stored into cuda textures for the evalution of the
    scalar field on GPU. HRBF_env handles CUDA memory has well as the
    computation of the HRBF weights. This class is supposed to be a wrapper
    for HRBF_env.

    @warning don't add more attributes without changing skeleton_tex.hpp
    (this implicit primitives is stored into texture)
*/
struct HermiteRBF
{
    enum {
        /// HRBF has not been initialized with a set of points a normals
        UNINITIALIZED = -1
    };


    IF_CUDA_DEVICE_HOST
    HermiteRBF() : _id(UNINITIALIZED)
    {  }

    void initialize();
    void clear();

    /// Does the hrbf has any samples to interpolate
    bool empty() const;

    /// Compute and store rbf reconstruction points and coefficients
    /// before calling this one must initialize the hrbf with initialize()
    void init_coeffs(const std::vector<Vec3_cu>& nodes,
                            const std::vector<Vec3_cu>& normals);
#if !defined(NO_CUDA)
    /// init HRBF from samples and user defined weights
    /// before calling this one must initialize the hrbf with initialize()
    void init_coeffs(const std::vector<Vec3_cu>& nodes,
                            const std::vector<Vec3_cu>& normals,
                            const std::vector<float4>&  weights);
#endif

    /// Sets the radius of the HRBF used to transform the potential field from
    /// global to compact
    void set_radius(float r);

    float get_radius() const;

    void get_samples(std::vector<Vec3_cu>& list) const;

    void get_normals(std::vector<Vec3_cu>& list) const;

    // =========================================================================
    /// @name Evaluation of the potential and gradient (compact support)
    // =========================================================================
    IF_CUDA_DEVICE_HOST
    float fngf(Vec3_cu& gf, const Point_cu& p) const;

    // =========================================================================
    /// @name Evaluation of the potential and gradient (global support)
    // =========================================================================
    IF_CUDA_DEVICE_HOST
    float fngf_global(Vec3_cu& gf, const Point_cu& p) const;

    /// @return id of the hrbf in HRBF_env namespace @see HRBF_env
    inline int get_id() const { return _id; }

private:

    // =========================================================================
    /// @name Attributes
    // =========================================================================

    /// needed to handle several instance of HermiteRbf with global textures
    int _id;
};

#endif // HERMITE_RBF_HPP__

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
#include "hrbf_phi_funcs.hpp"
#include "hrbf_data.hpp"
#include "hrbf_setup.hpp"

#include "hrbf_core.hpp" ///< This file must be compile with gcc

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

typedef HRBF_fit< float, 3, PHI_TYPE> HRBF_3f;

HRBF_3f* g_hrbf = 0;

typedef HRBF_3f::MatrixDD MatrixDD;
typedef HRBF_3f::Vector   Vector;
typedef HRBF_3f::MatrixXX MatrixDX;
typedef HRBF_3f::MatrixXX MatrixXX;
typedef HRBF_3f::VectorX  VectorX;

// Wrapper Tools ---------------------------------------------------------------

/// convert a MatrixDX into a newly allocated Vec3_cu array of
/// size MatriDX.cols()
static void matrixDX_to_Vec3_cu_array(const MatrixDX& mat, Vec3_cu*& vec)
{
    int nb_col = mat.cols();
    vec        = new Vec3_cu[nb_col];
    for(int i = 0; i < nb_col; i++)
    {
        vec[i].x = mat(0, i);
        vec[i].y = mat(1, i);
        vec[i].z = mat(2, i);
    }
}

// -----------------------------------------------------------------------------

/// Convert an VectorX of base type Scalar into a newly allocated
/// array of size VectorX.rows()
template<class Scalar>
void vectorX_to_array(const VectorX& vec, Scalar*& tab)
{
    int nb_row = vec.rows();
    tab        = new Scalar[nb_row];
    for(int i = 0; i < nb_row; i++)
        tab[i] = vec(i);
}

// End  Wrapper Tools ----------------------------------------------------------

void hermite_fit(const Vec3_cu* points,
                 const Vec3_cu* normals,
                 int size,
                 HRBF_coeffs& res)
{
    delete g_hrbf;
    g_hrbf = new HRBF_fit< float, 3, PHI_TYPE>();

    std::vector<Vector> vec_points, vec_normals;
    for(int i = 0; i < size; i++)
    {
        vec_points.push_back ( Vector(points [i].x, points [i].y, points [i].z));
        vec_normals.push_back( Vector(normals[i].x, normals[i].y, normals[i].z));
    }

    // Compute coeffs :
    g_hrbf->hermite_fit(vec_points, vec_normals);

    // return Coeffs :
    res.size = g_hrbf->_node_centers.cols();
    vectorX_to_array<float>  (g_hrbf->_alphas,       res.alphas      );
    matrixDX_to_Vec3_cu_array(g_hrbf->_betas,        res.betas       );
    matrixDX_to_Vec3_cu_array(g_hrbf->_node_centers, res.nodeCenters );
    res.normals = new Vec3_cu[size];
    memcpy(res.normals, normals, size*sizeof(Vec3_cu));
}

}// END RBFWrapper =============================================================

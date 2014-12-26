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
/// @file hrbf_core.hpp
/// @note header not compatible nvcc
/// @author Gael Guennebaud - gael.guennebaud@inria.fr

#ifndef HRBF_CORE_HPP__
#define HRBF_CORE_HPP__

#include <Eigen/LU>
#include <vector>
#include <iostream>

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

/// @brief fitting surface on a cloud point and evaluating the implicit surface
/// @tparam _Scalar : a base type float double...
/// @tparam _Dim : dimension of the ambient space
/// @tparam Rbf : the radial basis function used
/// @note see hrbf_setup.hpp for some examples of hrbf
template<typename _Scalar, int _Dim, typename Rbf>
class HRBF_fit
{
public:
    typedef _Scalar Scalar;
    enum { Dim = _Dim };

    typedef Eigen::Matrix<Scalar,Dim,Dim>                       MatrixDD;
    typedef Eigen::Matrix<Scalar,Dim,1>                         Vector;
    typedef Eigen::Matrix<Scalar,Dim,Eigen::Dynamic>            MatrixDX;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixXX;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1>              VectorX;

    HRBF_fit() {}

    // --------------------------------------------------------------------------

    void hermite_fit(const std::vector<Vector>& points,
                     const std::vector<Vector>& normals)
    {
        hermite_fit(points,normals,points);
    }

    void hermite_fit(const std::vector<Vector>& points,
                     const std::vector<Vector>& normals,
                     const std::vector<Vector>& nodes)
    {
        assert( points.size() == normals.size() );
        int nb_points           = points.size();
        int nb_nodes            = nodes.size();
        int nb_hrbf_constraints = (Dim+1)*nb_points;
        int nb_constraints      = nb_hrbf_constraints;
        int nb_coeffs           = (Dim+1)*nb_nodes;

        _node_centers.resize(Dim, nb_nodes);
        _betas.       resize(Dim, nb_nodes);
        _alphas.      resize(nb_nodes);

        // Assemble the "design" and "value" matrix and vector
        MatrixXX  D(nb_constraints, nb_coeffs);
        VectorX   f(nb_constraints);
        VectorX   x(nb_coeffs);

        // copy the node centers
        for(int i = 0; i < nb_nodes; ++i)
            _node_centers.col(i) = nodes[i];

        for(int i = 0; i < nb_points; ++i)
        {
            Vector p = points[i];
            Vector n = normals[i];

            int io = (Dim+1) * i;
            f(io) = 0;
            f.template segment<Dim>(io + 1) = n;

            for(int j = 0; j < nb_nodes; ++j)
            {
                int jo = (Dim + 1) * j;
                Vector diff = p - _node_centers.col(j);
                Scalar l = diff.norm();
                if( l == 0 ) {
                    D.template block<Dim+1,Dim+1>(io,jo).setZero();
                } else {
                    Scalar w    = Rbf::f(l);
                    Scalar dw_l = Rbf::df(l)/l;
                    Scalar ddw  = Rbf::ddf(l);
                    Vector g    = diff * dw_l;
                    D(io,jo) = w;
                    D.row(io).template segment<Dim>(jo+1) = g.transpose();
                    D.col(jo).template segment<Dim>(io+1) = g;
                    D.template block<Dim,Dim>(io+1,jo+1)  = (ddw - dw_l)/(l*l) * (diff * diff.transpose());
                    D.template block<Dim,Dim>(io+1,jo+1).diagonal().array() += dw_l;
                }
            }
        }

        if(nb_points == nb_nodes) x = D.lu().solve(f);
        else                      x = (D.transpose()*D).lu().solve(D.transpose()*f);

        Eigen::Map< Eigen::Matrix<Scalar,Dim+1,Eigen::Dynamic> > mx( x.data(), Dim + 1, nb_nodes);

        _alphas = mx.row(0);
        _betas  = mx.template bottomRows<Dim>();
    }

    // --------------------------------------------------------------------------

    /// evaluate potential at position 'x'
    Scalar eval(const Vector& x) const
    {
        Scalar ret = 0;
        int nb_nodes = _node_centers.cols();

        for(int i = 0; i < nb_nodes; ++i)
        {
            Vector diff = x - _node_centers.col(i);
            Scalar l    = diff.norm();

            if( l > 0 )
            {
                ret += _alphas(i) * Rbf::f(l);
                ret += _betas.col(i).dot( diff ) * Rbf::df(l) / l;
            }
        }
        return ret;
    }

    /// evaluate gradient ate position 'x'
    Vector grad(const Vector& x) const
    {
        Vector grad = Vector::Zero();
        int nb_nodes = _node_centers.cols();
        for(int i = 0; i < nb_nodes; i++)
        {
            Vector node  = _node_centers.col(i);
            Vector beta  = _betas.col(i);
            float  alpha = _alphas(i);
            Vector diff  = x - node;

            Vector diffNormalized = diff;
            float l =  diff.norm();

            if( l > 0.00001f)
            {
                diffNormalized.normalize();
                float dphi  = Rbf::df(l);
                float ddphi = Rbf::ddf(l);

                float alpha_dphi = alpha * dphi;

                float bDotd_l = beta.dot(diff)/l;
                float squared_l = diff.squaredNorm();

                grad += alpha_dphi * diffNormalized;
                grad += bDotd_l * (ddphi * diffNormalized - diff * dphi / squared_l) + beta * dphi / l ;
            }
        }
        return grad;
    }

    // --------------------------------------------------------------------------

    MatrixDX  _node_centers;
    VectorX   _alphas;
    MatrixDX  _betas;

}; // END HermiteRbfReconstruction Class =======================================

}// END RBFWrapper =============================================================

#endif // HRBF_CORE_HPP__


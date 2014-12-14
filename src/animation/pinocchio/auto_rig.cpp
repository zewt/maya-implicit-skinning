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
#include "auto_rig.hpp"

#include <vector>
#include <iostream>
#include <limits>


#if 0
// TODO: code in this #if to be deleted
//////////////////////////////////////
// ORIGINAL HEAT DIFFUSION USING TAUCS
// FROM POPOVIC 'AUTOMATIC RIGGING OF 3D CHARCHTERS'
void rig(const std::vector< Vec3_cu             >& vertices,
         const std::vector< std::vector<int>    >& edges,
         const std::vector< std::vector<double> >& boneDists,
         const std::vector< std::vector<bool>   >& boneVis,
         std::vector<std::vector<std::pair<int, double> > >& nzweights,
         double heat,
         int* nearest_bone,
         const std::vector<double>& nearest_bone_dist)
{
    int i, j;
    int nv       = vertices.size();
    int nb_bones = boneDists[0].size();

    //We have -Lw+Hw=HI, same as (H-L)w=HI, with (H-L)=DA (with D=diag(1./area))
    //so w = A^-1 * HI * D^-1
    std::vector<std::vector<std::pair<int, double> > > A(nv);
    std::vector<double> D(nv, 0.), H(nv, 0.);
    std::vector<int> closest(nv, -1);
    for(i = 0; i < nv; ++i)
    {
        Vec3_cu cPos = vertices[i];
        //get areas
        for(j = 0; j < (int)edges[i].size(); ++j)
        {
            int nj = (j + 1) % edges[i].size();

            Vec3_cu edge0 = vertices[edges[i][j] ] - cPos;
            Vec3_cu edge1 = vertices[edges[i][nj]] - cPos;

            D[i] += (edge0.cross(edge1)).norm();
        }
        D[i] = 1. / (1e-10 + D[i]);


            //get bones
            double minDist = std::numeric_limits<float>::infinity();
            for(j = 0; j < nb_bones; ++j) {
                if(boneDists[i][j] < minDist) {
                    closest[i] = j;
                    minDist = boneDists[i][j];
                }
            }

            // Bones equaly distant from vertex i are factored in H
            for(j = 0; j < nb_bones; ++j)
            {
                if(boneVis[i][j] && boneDists[i][j] <= minDist * 1.00001)
                {
                    double dist = 1e-8 + /*nearest_bone_dist[i];*/ boneDists[i][closest[i]];
                    H[i] += heat / (dist * dist);
                }
            }


        //get laplacian
        double sum = 0.;
        for(j = 0; j < (int)edges[i].size(); ++j) {
            int nj = (j + 1) % edges[i].size();
            int pj = (j + edges[i].size() - 1) % edges[i].size();

            Vec3_cu v1 = cPos                  - vertices[edges[i][pj]];
            Vec3_cu v2 = vertices[edges[i][j]] - vertices[edges[i][pj]];
            Vec3_cu v3 = cPos                  - vertices[edges[i][nj]];
            Vec3_cu v4 = vertices[edges[i][j]] - vertices[edges[i][nj]];

            double cot1 = (v1.dot(v2)) / (1e-6 + (v1.cross(v2)).norm() );
            double cot2 = (v3.dot(v4)) / (1e-6 + (v3.cross(v4)).norm() );

            sum += (cot1 + cot2);

            //check for triangular here because sum should be computed regardless
            if(edges[i][j] > i) continue;

            assert(edges[i][j] < nv);
            A[i].push_back(std::make_pair(edges[i][j], -cot1 - cot2));
        }

        A[i].push_back( std::make_pair(i, sum + H[i] / D[i]) );

        std::sort(A[i].begin(), A[i].end());
    }

    nzweights.resize(nv);

    SPDMatrix Am(A);
    LLTMatrix *Ainv = Am.factor();
    if(Ainv == 0) return;


    for(j = 0; j < nb_bones; ++j)
    {
        std::vector<double> rhs(nv, 0.);

        for(i = 0; i < nv; ++i)
        {
            if(boneVis[i][j] && boneDists[i][j] <= boneDists[i][closest[i]] * 1.00001)
            {
                rhs[i] = H[i] / D[i];
            }
        }

        Ainv->solve(rhs);
        for(i = 0; i < nv; ++i)
        {
            if(rhs[i] > 1.)
                rhs[i] = 1.; //clip just in case
            if(rhs[i] > 1e-3)
                nzweights[i].push_back(std::make_pair(j, rhs[i]));
        }
    }

    for(i = 0; i < nv; ++i)
    {
        double sum = 0.;
        for(j = 0; j < (int)nzweights[i].size(); ++j)
            sum += nzweights[i][j].second;

        for(j = 0; j < (int)nzweights[i].size(); ++j)
            nzweights[i][j].second /= sum;
    }

    delete Ainv;
    return;
}

#else

// -----------------------------------------------------------------------------

/// Are angles obtuse between e0 and e1 (meaning a < PI/2)
static bool check_obtuse(const Vec3_cu& e0,
                         const Vec3_cu& e1)
{
    return e0.normalized().dot( e1.normalized() ) >= 0.f;
}

// -----------------------------------------------------------------------------

/**
 * @brief Check if a triangle is obtuse (every angles < PI/2)
 * Here how edges must be defined :
   @code
      p0
      |\
      | \
      |  \
      |   \
      |____\
     p1     p2

    Vec3_cu e0 = p1 - p0;
    Vec3_cu e1 = p2 - p0;
    Vec3_cu e2 = p2 - p1;
   @endcode
 */
static bool check_obtuse(const Vec3_cu& e0,
                         const Vec3_cu& e1,
                         const Vec3_cu& e2)
{
    return check_obtuse(e0, e1) && check_obtuse(-e0, e2) && check_obtuse(-e1, -e2);
}

// -----------------------------------------------------------------------------

double mixed_voronoi_area(const Vec3_cu& pi,
                          const Vec3_cu& pj0,
                          const Vec3_cu& pj1)
{
    double area = 0.;
    Vec3_cu e0 = pj0 - pi ;
    Vec3_cu e1 = pj1 - pi;
    Vec3_cu e2 = pj1 - pj0;

    if( check_obtuse(e0, e1, e2) )
    {
        area = (1/8.) * (double)(e0.norm_squared() * (-e0).cotan( e2) +
                                 e1.norm_squared() * (-e1).cotan(-e2));
    }
    else
    {
        const double ta = (double)e0.cross( e1 ).norm() / 2.; // Tri area
        area = ta / (check_obtuse(e0, e1) ? 2. : 4.);
    }
    return area;
}

// -----------------------------------------------------------------------------

#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>


typedef Eigen::Triplet<double> ETriplet;
// declares a column-major sparse matrix type of double
typedef Eigen::SparseMatrix<double> SpMat;
using namespace Eigen;

// -----------------------------------------------------------------------------

void rig(const std::vector< Vec3_cu             >& vertices,
         const std::vector< std::vector<int>    >& edges,
         const std::vector< std::vector<double> >& boneDists,
         const std::vector< std::vector<bool>   >& boneVis,
         std::vector<std::vector<std::pair<int, double> > >& nzweights,
         double heat,
         int* ,
         const std::vector<double>& )
{
    bool state = false;
    if(state)
    {
        std::cout << "COMPUTE HEAT DIFFUSION WITH MY LAPLACIAN" << std::endl;
        // USING EIGEN AND MY LAPLACIAN ========================================
        int i, j;
        int nv       = vertices.size();
        int nb_bones = boneDists[0].size();

        // laplacian matrix
        MatrixXd L = MatrixXd::Constant(nv, nv, 0.);
        std::vector<double> D(nv, 0.), H(nv, 0.);

        std::vector<int> closest(nv, -1); // closest bone from the ith vertex
        for(i = 0; i < nv; ++i)
        {
            Vec3_cu cPos = vertices[i];

            //get bones
            double minDist = std::numeric_limits<float>::infinity();
            for(j = 0; j < nb_bones; ++j) {
                if(boneDists[i][j] < minDist) {
                    closest[i] = j;
                    minDist = boneDists[i][j];
                }
            }

            // Bones equaly distant from vertex i are factored in H
            for(j = 0; j < nb_bones; ++j)
            {
                if(boneVis[i][j] && boneDists[i][j] <= minDist * 1.00001)
                {
                    double dist = 1e-8 + boneDists[i][closest[i]];
                    H[i] += heat / (dist * dist);
                }
            }

            //get areas
            for(j = 0; j < (int)edges[i].size(); ++j)
            {
                int nj = (j + 1) % edges[i].size();

                #if 1
                Vec3_cu p0 = vertices[edges[i][j] ];
                Vec3_cu p1 = vertices[edges[i][nj]];
                D[i] += mixed_voronoi_area(cPos, p0, p1 );
                #else
                Vec3_cu edge0 = vertices[edges[i][j] ] - cPos;
                Vec3_cu edge1 = vertices[edges[i][nj]] - cPos;
                D[i] += (edge0.cross(edge1)).norm() / 2.f;
                #endif
            }
            D[i] = 1. / ((1e-10 + D[i]) * 2.f);

            //get laplacian
            double sum = 0.;
            for(j = 0; j < (int)edges[i].size(); ++j)
            {
                int nj = (j + 1) % edges[i].size();
                int pj = (j + edges[i].size() - 1) % edges[i].size();

                Vec3_cu v1 = cPos                  - vertices[edges[i][pj]];
                Vec3_cu v2 = vertices[edges[i][j]] - vertices[edges[i][pj]];
                Vec3_cu v3 = cPos                  - vertices[edges[i][nj]];
                Vec3_cu v4 = vertices[edges[i][j]] - vertices[edges[i][nj]];

                double cotan1 = (v1.dot(v2)) / (1e-6 + (v1.cross(v2)).norm() );
                double cotan2 = (v3.dot(v4)) / (1e-6 + (v3.cross(v4)).norm() );

                double w = (cotan1 + cotan2) * D[i];
                sum += w;

                L(i, edges[i][j]) = w;
            }

            L(i,i) = -sum;
        }

        L *= -1.;
        for(i = 0; i < nv; ++i)
            L(i,i) += H[i];

        ColPivHouseholderQR<MatrixXd> llt;
        //FullPivLU<MatrixXd> llt;
        llt.compute( L );

        nzweights.resize(nv);
        for(j = 0; j < nb_bones; ++j)
        {
            VectorXd rhs = VectorXd::Constant(nv, 0.);

            for(i = 0; i < nv; ++i)
            {
                if(boneVis[i][j] && boneDists[i][j] <= boneDists[i][closest[i]] * 1.00001)
                    rhs(i) = H[i];
            }

            VectorXd res = llt.solve( rhs );

            for(i = 0; i < nv; ++i)
            {
                if(res(i) > 1.)   res(i) = 1.; //clip just in case

                if(res(i) > 1e-3) nzweights[i].push_back(std::make_pair(j, res(i)));
            }
        }

        for(i = 0; i < nv; ++i)
        {
            double sum = 0.;
            for(j = 0; j < (int)nzweights[i].size(); ++j)
                sum += nzweights[i][j].second;

            for(j = 0; j < (int)nzweights[i].size(); ++j)
                nzweights[i][j].second /= sum;
        }

        return;
    }
    else
    {
        // USING EIGEN AND ORIGINAL LAPLACIAN ==================================
        int i, j;
        int nv       = vertices.size();
        int nb_bones = boneDists[0].size();

        //We have -Lw+Hw=HI, same as (H-L)w=HI, with (H-L)=DA (with D=diag(1./area))
        //so w = A^-1 * HI * D^-1
        std::vector<std::vector<std::pair<int, double> > > A(nv);
        std::vector<double> D(nv, 0.), H(nv, 0.);
        std::vector<int> closest(nv, -1);
        int acc = 0;
        for(i = 0; i < nv; ++i)
        {
            Vec3_cu cPos = vertices[i];
            //get areas
            for(j = 0; j < (int)edges[i].size(); ++j)
            {
                int nj = (j + 1) % edges[i].size();

                Vec3_cu edge0 = vertices[edges[i][j] ] - cPos;
                Vec3_cu edge1 = vertices[edges[i][nj]] - cPos;

                D[i] += (edge0.cross(edge1)).norm();
            }
            D[i] = 1. / (1e-10 + D[i]);


            //get bones
            double minDist = std::numeric_limits<float>::infinity();
            for(j = 0; j < nb_bones; ++j) {
                if(boneDists[i][j] < minDist) {
                    closest[i] = j;
                    minDist = boneDists[i][j];
                }
            }

            // Bones equaly distant from vertex i are factored in H
            for(j = 0; j < nb_bones; ++j)
            {
                if(boneVis[i][j] && boneDists[i][j] <= minDist * 1.00001)
                {
                    double dist = 1e-8 + boneDists[i][closest[i]];
                    H[i] += heat / (dist * dist);
                }
            }

            // get laplacian
            double sum = 0.;
            for(j = 0; j < (int)edges[i].size(); ++j) {
                int nj = (j + 1) % edges[i].size();
                int pj = (j + edges[i].size() - 1) % edges[i].size();

                Vec3_cu v1 = cPos                  - vertices[edges[i][pj]];
                Vec3_cu v2 = vertices[edges[i][j]] - vertices[edges[i][pj]];
                Vec3_cu v3 = cPos                  - vertices[edges[i][nj]];
                Vec3_cu v4 = vertices[edges[i][j]] - vertices[edges[i][nj]];

                double cot1 = (v1.dot(v2)) / (1e-6 + (v1.cross(v2)).norm() );
                double cot2 = (v3.dot(v4)) / (1e-6 + (v3.cross(v4)).norm() );

                sum += (cot1 + cot2);

                //check for triangular here because sum should be computed regardless
                if(edges[i][j] > i) continue;

                assert(edges[i][j] < nv);
                A[i].push_back(std::make_pair(edges[i][j], -cot1 - cot2));
                acc++;
            }

            A[i].push_back( std::make_pair(i, sum + H[i] / D[i]) );
            acc++;

            std::sort(A[i].begin(), A[i].end());
        }
        nzweights.resize(nv);


#if 0
        // Solving with plain matrix (slower but more robust)
        MatrixXd L = MatrixXd::Constant(nv, nv, 0.);
        for(int u = 0; u < nv; ++u) {
            for(unsigned v = 0; v < A[u].size(); ++v) {
                L(u, A[u][v].first) = A[u][v].second;
                if(A[u][v].first != u)
                    L(A[u][v].first, u) = A[u][v].second;
            }
        }

        //LLT<MatrixXd, Lower> llt;
        ColPivHouseholderQR<MatrixXd> llt;
        llt.compute( L );
#else
        // Faster solving with sparse matrix:
        std::cout << "Trying to allocate: "
                  << (float)(acc*sizeof(ETriplet)) / 1024.f / 1024.f
                  << "Mb. To compute the heat diffusion"
                  << std::endl;

        std::vector<ETriplet> tripletList;
        tripletList.resize( acc );
        {
            int acc = 0;
            for(int u = 0; u < nv; ++u) {
                for(unsigned v = 0; v < A[u].size(); ++v) {
                    tripletList[acc] = ETriplet(u, A[u][v].first, A[u][v].second);
                    acc++;
                }
            }
        }
        SpMat L(nv, nv);
        L.setFromTriplets(tripletList.begin(), tripletList.end());

        SimplicialLLT<SpMat, Lower> llt;
        llt.compute( L );
#endif

        for(j = 0; j < nb_bones; ++j)
        {
            VectorXd rhs = VectorXd::Constant(nv, 0.);

            for(i = 0; i < nv; ++i)
            {
                if(boneVis[i][j] && boneDists[i][j] <= boneDists[i][closest[i]] * 1.00001)
                {
                    rhs(i) = H[i] / D[i];
                }
            }

            VectorXd res = llt.solve( rhs );

            for(i = 0; i < nv; ++i)
            {
                if(res(i) > 1.) res(i) = 1.; //clip just in case

                if(res(i) > 1e-3)
                    nzweights[i].push_back(std::make_pair(j, res(i)));
            }
        }

        for(i = 0; i < nv; ++i)
        {
            double sum = 0.;
            for(j = 0; j < (int)nzweights[i].size(); ++j)
                sum += nzweights[i][j].second;

            for(j = 0; j < (int)nzweights[i].size(); ++j)
                nzweights[i][j].second /= sum;
        }
        return;
    }
}


#endif



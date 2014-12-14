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
#include "vcg_mesh.hpp"

#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/append.h>

// =============================================================================
namespace vcg {
// =============================================================================


MyMesh::MyMesh() : vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> >()
{
    bbox = Box3<ScalarType>();
}

// -----------------------------------------------------------------------------

MyMesh::MyMesh(const MyMesh& m){
    this->Clear();
    // Since append is not using const iterators for m I'm forced to use
    // the ugly const_cast(). shame I know ...
    tri::Append<MyMesh, MyMesh>::Mesh(*this, const_cast<MyMesh&>(m));
}

// -----------------------------------------------------------------------------

void MyMesh::concat(const float* verts,
                    const int* triangles,
                    int nb_vert,
                    int nb_face)

{
    vcg::tri::Allocator<MyMesh>::AddVertices(*this, nb_vert);
    vcg::tri::Allocator<MyMesh>::AddFaces   (*this, nb_face);

    std::vector<VertexPointer> ivp( nb_vert );
    VertexIterator vi = this->vert.end();
    for(int i = (nb_vert-1); i >= 0; --i)
    {
        --vi;
        ivp[i] = &*vi;
        (*vi).P() = CoordType(verts[i*3], verts[i*3+1], verts[i*3+2]);
    }

    FaceIterator fi = this->face.end();
    for(int i = (nb_face-1); i >= 0; --i)
    {
        --fi;
        (*fi).V(0) = ivp[ triangles[i*3 + 0] ];
        (*fi).V(1) = ivp[ triangles[i*3 + 1] ];
        (*fi).V(2) = ivp[ triangles[i*3 + 2] ];
    }
}

// -----------------------------------------------------------------------------

void MyMesh::set_normals(const float* normals)
{
    const int nb_vert = this->vert.size();
    vcg::MyMesh::VertexIterator vi = this->vert.begin();
    for(int i = 0; i < nb_vert; ++i, ++vi)
    {
        (*vi).N() = NormalType(normals[i*3 + 0], normals[i*3 + 1], normals[i*3 + 2]);
        /*
        (*vi).N().X() = normals[i*3 + 0];
        (*vi).N().Y() = normals[i*3 + 1];
        (*vi).N().Z() = normals[i*3 + 2];
        */
    }
}

// -----------------------------------------------------------------------------

void MyMesh::update_bb()
{
    vcg::tri::UpdateBounding<MyMesh>::Box( *this );
}

// -----------------------------------------------------------------------------

void fill_vcg_mesh(MyMesh& vcg_mesh,
                   const float* verts,
                   const int* triangles,
                   int nb_vert,
                   int nb_face)

{
    vcg_mesh.Clear();
    vcg::tri::Allocator<MyMesh>::AddVertices(vcg_mesh, nb_vert);
    vcg::tri::Allocator<MyMesh>::AddFaces   (vcg_mesh, nb_face);

    std::vector<MyMesh::VertexPointer> ivp( nb_vert );
    MyMesh::VertexIterator vi = vcg_mesh.vert.begin();
    for(int i = 0; i < nb_vert; ++i, ++vi)
    {
        ivp[i] = &*vi;
        (*vi).P() = MyMesh::CoordType(verts[i*3], verts[i*3+1], verts[i*3+2]);
    }

    MyMesh::FaceIterator fi = vcg_mesh.face.begin();
    for(int i = 0; i < nb_face; ++i, ++fi)
    {
        (*fi).V(0) = ivp[ triangles[i*3 + 0] ];
        (*fi).V(1) = ivp[ triangles[i*3 + 1] ];
        (*fi).V(2) = ivp[ triangles[i*3 + 2] ];
    }
}

// -----------------------------------------------------------------------------

void fill_mesh(Mesh*& m, const MyMesh& vcg_mesh)
{
    delete m;

    const int nb_vert = vcg_mesh.vert.size();
    const int nb_tri  = vcg_mesh.face.size();

    std::vector<int>   tri (nb_tri  * 3);
    std::vector<float> vert(nb_vert * 3);

    MyMesh::ConstVertexIterator vi = vcg_mesh.vert.begin();
    std::map<MyMesh::ConstVertexPointer, int> map_idx_addr;
    for(int i = 0; i < nb_vert; ++i, ++vi)
    {
        MyMesh::CoordType coord = (*vi).P();
        vert[i*3 + 0] = coord.X();
        vert[i*3 + 1] = coord.Y();
        vert[i*3 + 2] = coord.Z();

        map_idx_addr[ &(*vi) ] = i;
    }

    MyMesh::ConstFaceIterator fi = vcg_mesh.face.begin();
    for(int i = 0; i < nb_tri; ++i, ++fi)
    {
        tri[i*3 + 0] = map_idx_addr[ (*fi).V(0) ];
        tri[i*3 + 1] = map_idx_addr[ (*fi).V(1) ];
        tri[i*3 + 2] = map_idx_addr[ (*fi).V(2) ];
    }
    m = new Mesh(tri, vert);
}

// -----------------------------------------------------------------------------


// =============================================================================
namespace MyAlgorithms {
// =============================================================================


void poison_disk_sampling(MyMesh& vcg_mesh,
                          const Poison_setup& params,
                          MyMesh& samples)
{

    // Parameters
    float rad     = params._radius;
    int   nb_samp = params._nb_samples;
    int   mc_rate = params._montecarlo_rate;

    bool sub_sample      = params._sub_sampling;
    bool approx_geodesic = params._approx_geodesic_dist;
    bool refine          = params._refine;
    // end parameters

    // check parameters
    if( rad < 0.00001f && nb_samp == 0)
    {
        std::cerr << "ERROR poisson disk sampling:";
        std::cerr << "you must specify a maximal radius";
        std::cerr << "or a minimal number of samples";
    }

    if(rad <= 0) rad     = vcg::tri::SurfaceSampling<MyMesh, MySampler>::ComputePoissonDiskRadius( vcg_mesh, nb_samp );
    else         nb_samp = vcg::tri::SurfaceSampling<MyMesh, MySampler>::ComputePoissonSampleNum ( vcg_mesh, rad     );

    printf("Computing %i Poisson Samples for an expected radius of %f", nb_samp, rad);

    // first of all generate montecarlo samples for fast lookup
    MyMesh* presampledMesh = &vcg_mesh;

    // this mesh is used only if we need real poisson sampling
    // (and therefore we need to choose points different from
    // the starting mesh vertices)
    MyMesh montecarloMesh;

    if( !sub_sample )
    {
        MySampler sampler( &montecarloMesh );
        vcg::tri::SurfaceSampling<MyMesh, MySampler>::Montecarlo(vcg_mesh, sampler, nb_samp * mc_rate);

        montecarloMesh.bbox = vcg_mesh.bbox; // we want the same bounding box
        presampledMesh = &montecarloMesh;

        printf("Generated %i Montecarlo Samples", montecarloMesh.vn);
    }


    vcg::tri::SurfaceSampling<MyMesh, MySampler>::PoissonDiskParam pp;

    if( refine )
    {
        pp.preGenFlag = true;
        pp.preGenMesh = &vcg_mesh;
    }

    pp.geodesicDistanceFlag = approx_geodesic;

    {
        MySampler sampler( &samples );
        vcg::tri::SurfaceSampling<MyMesh, MySampler>::PoissonDiskPruning(vcg_mesh, sampler, *presampledMesh, rad, pp);
        //vcg::tri::SurfaceSampling<MyMesh,MySampler>::PoissonDisk(vcg_mesh, mps, *presampledMesh, radius,pp);

        samples.update_bb();
        printf("Sampling created a new mesh of %i points", samples.vn);
    }

}

}// END NAMESPACE MyAlgorithms =================================================

}// END NAMESPACE VCG ==========================================================

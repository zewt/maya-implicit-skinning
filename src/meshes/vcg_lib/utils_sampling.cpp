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
#include "utils_sampling.hpp"

#include "vcg_mesh.hpp"

// =============================================================================
namespace Utils_sampling {
// =============================================================================

void poisson_disk(float radius,
                  int nb_samples,
                  const std::vector<Vec3_cu>& verts,
                  const std::vector<Vec3_cu>& nors,
                  const std::vector<int>& tris,
                  std::vector<Vec3_cu>& samples_pos,
                  std::vector<Vec3_cu>& samples_nor)
{
    assert(verts.size() == nors.size());
    assert(verts.size() > 0);
    assert(tris.size() > 0);

    vcg::MyMesh vcg_mesh, sampler;
    vcg_mesh.concat((float*)&(verts[0]), (int*)&(tris[0]), verts.size(), tris.size()/3);
    vcg_mesh.set_normals( (float*)&(nors[0]) );
    vcg_mesh.update_bb();

    vcg::MyAlgorithms::Poison_setup pp;
    pp._radius = radius;
    pp._nb_samples = nb_samples;
    pp._approx_geodesic_dist = true;
    vcg::MyAlgorithms::poison_disk_sampling(vcg_mesh, pp, sampler);

    const int nb_vert = sampler.vert.size();
    samples_pos.clear();
    samples_nor.clear();
    samples_pos.resize( nb_vert );
    samples_nor.resize( nb_vert );
    vcg::MyMesh::VertexIterator vi = sampler.vert.begin();
    for(int i = 0; i < nb_vert; ++i, ++vi)
    {
        vcg::MyMesh::CoordType  p = (*vi).P();
        vcg::MyMesh::NormalType n = (*vi).N();
        samples_pos[i] = Vec3_cu(p.X(), p.Y(), p.Z());
        samples_nor[i] = Vec3_cu(n.X(), n.Y(), n.Z());
    }

    // sampler.get_vertices( samples_pos );

}

}// END UTILS_SAMPLING NAMESPACE ===============================================

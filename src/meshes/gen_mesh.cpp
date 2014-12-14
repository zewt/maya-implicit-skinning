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
#include "gen_mesh.hpp"

#include <cmath>
#include <cassert>

struct Vert{
    float x, y, z;
};

// =============================================================================
namespace Gen_mesh {
// =============================================================================

Tri_Mesh_data* cylinder(float length,
                        float radius,
                        int nb_slice,
                        int res,
                        bool cap)
{
    assert(res > 3);
    const int nb_vert = res*(2+nb_slice) + (cap ? 2 : 0);
    // Compute triangles index
    const int nb_tri  = res*2*(nb_slice+1) + (cap ? res*2 : 0);

    Tri_Mesh_data* cylinder = new Tri_Mesh_data(nb_vert, nb_tri);

    float* vert       = cylinder->vertex;
    float* normals    = cylinder->normals;
    int*   tri        = cylinder->index;

    // Compute vertex position of the body
    const float step_teta = 2.f * M_PI / (float)res;
    const float step_z    = length / (float)nb_slice;
    int vert_idx = 0;
    for(int z = 0; z < nb_slice+2; z++){
        for(int i = 0; i < res; i++){
            const float x = cosf(i*step_teta);
            const float y = sinf(i*step_teta);
            vert[vert_idx*3  ] = radius * x;
            vert[vert_idx*3+1] = radius * y;
            vert[vert_idx*3+2] = z*step_z;
            // Normalized normals :
            normals[vert_idx*3  ] = x;
            normals[vert_idx*3+1] = y;
            normals[vert_idx*3+2] = 0.f;
            vert_idx++;
        }
    }

    // add vertex caps
    if(cap){
        // first cap
        vert[vert_idx*3  ] = 0.f;
        vert[vert_idx*3+1] = 0.f;
        vert[vert_idx*3+2] = -radius*0.5f;
        normals[vert_idx*3  ] = 0.f;
        normals[vert_idx*3+1] = 0.f;
        normals[vert_idx*3+2] = -1.f;
        vert_idx++;
        // second cap
        vert[vert_idx*3  ] = 0.f;
        vert[vert_idx*3+1] = 0.f;
        vert[vert_idx*3+2] = ((float)(nb_slice+1)*step_z) + radius*0.5f;
        normals[vert_idx*3  ] = 0.f;
        normals[vert_idx*3+1] = 0.f;
        normals[vert_idx*3+2] = 1.f;
        vert_idx++;
    }
    assert(vert_idx == nb_vert);

    // Add body triangles
    int tri_idx = 0;
    for(int i = 0; i<nb_slice+1; i++){
        for(int t = 0; t < res; t++){
            tri[tri_idx*3  ] = i*res + t;
            tri[tri_idx*3+1] = i*res + (t+1)%res;
            tri[tri_idx*3+2] = (i+1)*res + t;
            tri_idx++;
            tri[tri_idx*3  ] = i*res + (t+1)%res;
            tri[tri_idx*3+1] = (i+1)*res + t;
            tri[tri_idx*3+2] = (i+1)*res + (t+1)%res;
            tri_idx++;
        }
    }

    // add caps triangles
    for(int i = 0; (i < res) && cap; i++){
        tri[tri_idx*3  ] = nb_vert-2;
        tri[tri_idx*3+1] = i;
        tri[tri_idx*3+2] = (i+1)%res;
        tri_idx++;
    }
    for(int i = 0; (i < res) && cap; i++){
        tri[tri_idx*3  ] = nb_vert-1;
        tri[tri_idx*3+1] = (nb_vert-2-res) + i;
        tri[tri_idx*3+2] = (nb_vert-2-res) + (i+1) % res;
        tri_idx++;
    }
    assert(tri_idx == nb_tri);

#ifndef NDEBUG
    for(int i = 0; i < nb_tri*3; i++)
        assert(tri[i] < (int)nb_vert);
#endif

    return cylinder;
}

// -----------------------------------------------------------------------------

Tri_Mesh_data* sphere(float radius, int res)
{
    assert(res>1);

    const int nb_vert = res*(res-1) + 2;
    const int nb_tri  = res*2*(res-2) + res*2;

    Tri_Mesh_data* sphere = new Tri_Mesh_data(nb_vert, nb_tri);

    const float longitude = (2.f*M_PI) / (float)res;
    const float latitude  = M_PI       / (float)res;
    int vert_idx = 0;
    // Add points to body
    for(float v = (-M_PI/2.f)+latitude; v < (M_PI/2.f); v += latitude){
        for(float u = 0; u < (2.f*M_PI); u += longitude ){
            float va, vb, vc;
            va = radius * cosf(u) * cosf(v);
            vb = radius * sinf(u) * cosf(v);
            vc = radius * sinf(v);

            sphere->vertex[vert_idx*3    ] = va;
            sphere->vertex[vert_idx*3 + 1] = vb;
            sphere->vertex[vert_idx*3 + 2] = vc;
            // Normalized normals :
            sphere->normals[vert_idx*3    ] = cosf(u) * cosf(v);
            sphere->normals[vert_idx*3 + 1] = sinf(u) * cosf(v);
            sphere->normals[vert_idx*3 + 2] = sinf(v);

            vert_idx++;
        }
    }
    // Add points to top and bottom of the sphere:
    sphere->vertex[vert_idx*3    ] = 0.f;
    sphere->vertex[vert_idx*3 + 1] = 0.f;
    sphere->vertex[vert_idx*3 + 2] = radius;

    sphere->normals[vert_idx*3    ] = 0.f;
    sphere->normals[vert_idx*3 + 1] = 0.f;
    sphere->normals[vert_idx*3 + 2] = 1.f;
    vert_idx++;

    sphere->vertex[vert_idx*3    ] = 0.f;
    sphere->vertex[vert_idx*3 + 1] = 0.f;
    sphere->vertex[vert_idx*3 + 2] = -radius;

    sphere->normals[vert_idx*3    ] = 0.f;
    sphere->normals[vert_idx*3 + 1] = 0.f;
    sphere->normals[vert_idx*3 + 2] = -1.f;
    vert_idx++;

    assert(vert_idx == nb_vert);
    // Generate triangle index
    // Add body triangles
    int tri_idx = 0;
    for(int i = 0; i<res-2; i++){
        for(int t = 0; t < res; t++){
            sphere->index[tri_idx*3  ] = i*res + t;
            sphere->index[tri_idx*3+1] = i*res + (t+1)%res;
            sphere->index[tri_idx*3+2] = (i+1)*res + t;
            tri_idx++;
            sphere->index[tri_idx*3  ] = (i+1)*res + (t+1)%res;
            sphere->index[tri_idx*3+1] = (i+1)*res + t;
            sphere->index[tri_idx*3+2] = i*res + (t+1)%res;
            tri_idx++;
            assert(tri_idx < nb_tri);
        }
    }

    // add bottom and top triangles
    for(int i = 0; i < res; i++){
        sphere->index[tri_idx*3  ] = (i+1)%res;
        sphere->index[tri_idx*3+1] = i;
        sphere->index[tri_idx*3+2] = nb_vert-1;
        tri_idx++;
    }
    for(int i = 0; i < res; i++){
        sphere->index[tri_idx*3  ] = nb_vert-2;
        sphere->index[tri_idx*3+1] = (nb_vert-2-res) + i;
        sphere->index[tri_idx*3+2] = (nb_vert-2-res) + (i+1) % res;
        tri_idx++;
    }
    assert(tri_idx == nb_tri);


#ifndef NDEBUG
    for(int i = 0; i < nb_tri*3; i++){
        assert(sphere->index[i] < (int)nb_vert);
    }
#endif

    return sphere;
}

// -----------------------------------------------------------------------------

Tri_Mesh_data* cone(float radius,
                float height,
                int res)

{
    assert(res>1);

    const int nb_vert = (res * res) + 2;
    const int nb_tri  = res * 2 * (res-1) + res*2;

    Tri_Mesh_data* cone = new Tri_Mesh_data(nb_vert, nb_tri);

    // Parametric equation of a cone :
    // x = ((height-u) / height) radius * cos( teta )
    // y = ((height-u) / height) radius * sin( teta )
    // y = u
    // with u [0; height] and teta [0; 2PI]
    const float step_teta = (2.f*M_PI) / (float)res;
    const float step_u    = height   / (float)res;
    int vert_idx = 0;
    // Add points to body
    // TODO: use res to iterate with for
    for(int u = 0; u < res; u++){
        for(int teta = 0; teta < res; teta++){
            float va, vb, vc;
            float i_u    = u * step_u;
            float i_teta = teta * step_teta;
            va = ((height-i_u) / height) * cosf( i_teta );
            vb = ((height-i_u) / height) * sinf( i_teta );
            vc = i_u;
            cone->vertex[vert_idx*3    ] = va * radius;
            cone->vertex[vert_idx*3 + 1] = vb * radius;
            cone->vertex[vert_idx*3 + 2] = vc * radius;
            // Normalized normals :
            cone->normals[vert_idx*3    ] = 2.f * va;
            cone->normals[vert_idx*3 + 1] = 2.f * vb;
            cone->normals[vert_idx*3 + 2] = 2.f * (radius/height)*(radius/height);

            vert_idx++;
        }
    }
    // Add points to top and bottom of the cone:
    cone->vertex[vert_idx*3    ] = 0.f;
    cone->vertex[vert_idx*3 + 1] = 0.f;
    cone->vertex[vert_idx*3 + 2] = 0.f;

    cone->normals[vert_idx*3    ] = 0.f;
    cone->normals[vert_idx*3 + 1] = 0.f;
    cone->normals[vert_idx*3 + 2] = -1.f;
    vert_idx++;

    cone->vertex[vert_idx*3    ] = 0.f;
    cone->vertex[vert_idx*3 + 1] = 0.f;
    cone->vertex[vert_idx*3 + 2] = step_u * res; // for numeric instabilities

    cone->normals[vert_idx*3    ] = 0.f;
    cone->normals[vert_idx*3 + 1] = 0.f;
    cone->normals[vert_idx*3 + 2] = 1.f;
    vert_idx++;

    assert(vert_idx == nb_vert);
    // Generate triangle index
    // Add body triangles
    int tri_idx = 0;
    for(int i = 0; i<(res-1); i++){
        for(int t = 0; t < res; t++){
            cone->index[tri_idx*3  ] = i*res + t;
            cone->index[tri_idx*3+1] = i*res + (t+1)%res;
            cone->index[tri_idx*3+2] = (i+1)*res + t;
            tri_idx++;
            cone->index[tri_idx*3  ] = i*res + (t+1)%res;
            cone->index[tri_idx*3+1] = (i+1)*res + t;
            cone->index[tri_idx*3+2] = (i+1)*res + (t+1)%res;
            tri_idx++;
            assert(tri_idx < nb_tri);
        }
    }

    // add bottom and top triangles
    for(int i = 0; i < res; i++){
        cone->index[tri_idx*3  ] = nb_vert-2;
        cone->index[tri_idx*3+1] = i;
        cone->index[tri_idx*3+2] = (i+1)%res;
        tri_idx++;
    }
    for(int i = 0; i < res; i++){
        cone->index[tri_idx*3  ] = nb_vert-1;
        cone->index[tri_idx*3+1] = (nb_vert-2-res) + i;
        cone->index[tri_idx*3+2] = (nb_vert-2-res) + (i+1) % res;
        tri_idx++;
    }
    assert(tri_idx == nb_tri);


#ifndef NDEBUG
    for(int i = 0; i < nb_tri*3; i++){
        assert(cone->index[i] < (int)nb_vert);
    }
#endif

    return cone;
}

// -----------------------------------------------------------------------------

Quad_Mesh_data* cube()
{
    const int nb_vert = 24;
    const int nb_quad = 6;

    Quad_Mesh_data* cuboid = new Quad_Mesh_data(nb_vert, nb_quad);

    int quad_idx = 0;
    int vert_idx = 0;

    // Bottom
    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = -1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = -1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 1] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = -1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 2] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = -1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 3] = vert_idx;
    vert_idx++; quad_idx++;

    // Top
    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 1] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 2] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 1.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 3] = vert_idx;
    vert_idx++; quad_idx++;

    // Left
    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = -1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = -1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 1] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = -1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 2] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = -1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 3] = vert_idx;
    vert_idx++; quad_idx++;

    // Right
    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 1] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 2] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 1.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 0.f;
    cuboid->index[quad_idx*4 + 3] = vert_idx;
    vert_idx++; quad_idx++;

    // Rear
    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = -1.f;
    cuboid->index[quad_idx*4] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = -1.f;
    cuboid->index[quad_idx*4 + 1] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = -1.f;
    cuboid->index[quad_idx*4 + 2] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 0.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = -1.f;
    cuboid->index[quad_idx*4 + 3] = vert_idx;
    vert_idx++; quad_idx++;

    // Front
    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 1.f;
    cuboid->index[quad_idx*4] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 0.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 1.f;
    cuboid->index[quad_idx*4 + 1] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 1.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 1.f;
    cuboid->index[quad_idx*4 + 2] = vert_idx;
    vert_idx++;

    cuboid->vertex[vert_idx*3] = 1.f; cuboid->vertex[vert_idx*3+1] = 0.f; cuboid->vertex[vert_idx*3+2] = 1.f;
    cuboid->normals[vert_idx*3] = 0.f; cuboid->normals[vert_idx*3+1] = 0.f; cuboid->normals[vert_idx*3+2] = 1.f;
    cuboid->index[quad_idx*4 + 3] = vert_idx;
    vert_idx++; quad_idx++;

    assert(nb_vert == vert_idx);
    assert(nb_quad == quad_idx);

    return cuboid;
}

// -----------------------------------------------------------------------------

Line_data* circle(float radius, int res, float diameter)
{
    assert(res > 3);
    const int nb_vert = res;
    const int nb_line = res;

    Line_data* circle = new Line_data(nb_vert, nb_line);

    float* vert    = circle->vertex;
    float* normals = circle->normals;
    int*   lines   = circle->index;

    // Compute vertex position of the body
    const float step_teta = diameter / (float)res;
    for(int i = 0; i < res; i++){
        const float x = cosf(i*step_teta);
        const float y = sinf(i*step_teta);
        vert[i*3  ] = radius * x;
        vert[i*3+1] = radius * y;
        vert[i*3+2] = 0.f;
        // Normalized normals :
        normals[i*3  ] = x;
        normals[i*3+1] = y;
        normals[i*3+2] = 0.f;
        // Line index
        lines[i*2  ] = i;
        lines[i*2+1] = (i+1)%res;
    }

    return circle;
}

// -----------------------------------------------------------------------------

Line_data* grid(float width, float height, int res_x, int res_y)
{
    const int nb_vert = (res_y+1)*2 + (res_x+1)*2;
    const int nb_line = (res_y+1)   + (res_x+1);

    Line_data* grid = new Line_data(nb_vert, nb_line);

    float*        vert    = grid->vertex;
    float*        normals = grid->normals;
    int* lines   = grid->index;

    const float step_x = width  / res_x;
    const float step_y = height / res_y;
    for(int i = 0; i < (res_y+1); i++)
    {
        const float y = i*step_y - (height/2.f);
        const int idx_0 = i*2;
        const int idx_1 = i*2+1;

        vert[idx_0*3  ] = -(width/2.f);
        vert[idx_0*3+1] = y;
        vert[idx_0*3+2] = 0.f;

        vert[idx_1*3  ] = (width/2.f);
        vert[idx_1*3+1] = y;
        vert[idx_1*3+2] = 0.f;

        // Normalized normals :
        normals[idx_0*3  ] = 0.f;
        normals[idx_0*3+1] = 0.f;
        normals[idx_0*3+2] = 1.f;

        normals[idx_1*3  ] = 0.f;
        normals[idx_1*3+1] = 0.f;
        normals[idx_1*3+2] = 1.f;

        // Line index
        lines[idx_0] = idx_0;
        lines[idx_1] = idx_1;
    }

    for(int i = 0; i < (res_x+1); i++)
    {
        const float x = i*step_x - (width/2.f);
        const int idx_0 = (i+(res_y+1)) * 2;
        const int idx_1 = (i+(res_y+1)) * 2 + 1;

        vert[idx_0*3  ] = x;
        vert[idx_0*3+1] = -(height/2.f);
        vert[idx_0*3+2] = 0.f;

        vert[idx_1*3  ] = x;
        vert[idx_1*3+1] = (height/2.f);
        vert[idx_1*3+2] = 0.f;

        // Normalized normals :
        normals[i*3  ] = 0.f;
        normals[i*3+1] = 0.f;
        normals[i*3+2] = 1.f;

        normals[i*3  ] = 0.f;
        normals[i*3+1] = 0.f;
        normals[i*3+2] = 1.f;

        // Line index
        lines[idx_0] = idx_0;
        lines[idx_1] = idx_1;
    }

    return grid;
}


// -----------------------------------------------------------------------------

Line_data* cylinder_cage(float length, float radius, int caps_res, int body_res)
{

    assert(caps_res > 3);

    Line_data* circ    = circle(radius, caps_res);

    const int nb_vert = body_res*2 + circ->nb_vert*2;
    const int nb_line = body_res + circ->nb_line*2;

    Line_data* cyl_cage = new Line_data(nb_vert, nb_line);

    float* vert    = cyl_cage->vertex;
    float* normals = cyl_cage->normals;
    int* lines     = cyl_cage->index;

    // Build body cage by hand
    int vidx = 0; // vert index
    int lidx = 0; // line index

    const float step_teta = 2.f * M_PI / (float)body_res;
    for(int i = 0; i < body_res; i++){
        const float x = cosf(i*step_teta);
        const float y = sinf(i*step_teta);
        vert[vidx*3  ] = radius * x;
        vert[vidx*3+1] = radius * y;
        vert[vidx*3+2] = 0.f;
        // Normalized normals :
        normals[vidx*3  ] = x;
        normals[vidx*3+1] = y;
        normals[vidx*3+2] = 0.f;
        lines[lidx*2] = vidx;
        vidx++;

        vert[vidx*3  ] = radius * x;
        vert[vidx*3+1] = radius * y;
        vert[vidx*3+2] = length;
        // Normalized normals :
        normals[vidx*3  ] = x;
        normals[vidx*3+1] = y;
        normals[vidx*3+2] = 0.f;
        lines[lidx*2+1] = vidx;
        lidx++;
        vidx++;
    }


    int off = vidx;
    for(int i = 0; i < circ->nb_vert; i++){
        vert[vidx*3  ] = circ->vertex[i*3  ];
        vert[vidx*3+1] = circ->vertex[i*3+1];
        vert[vidx*3+2] = circ->vertex[i*3+2];
        normals[vidx*3  ] = circ->normals[i*3  ];
        normals[vidx*3+1] = circ->normals[i*3+1];
        normals[vidx*3+2] = circ->normals[i*3+2];
        vidx++;
    }
    for(int i = 0; i < circ->nb_line; i++){
        lines[lidx*2  ] = circ->index[i*2  ]+off;
        lines[lidx*2+1] = circ->index[i*2+1]+off;
        lidx++;
    }
    // second circle
    off = vidx;
    for(int i = 0; i < circ->nb_vert; i++){
        vert[vidx*3  ] = circ->vertex[i*3  ];
        vert[vidx*3+1] = circ->vertex[i*3+1];
        vert[vidx*3+2] = circ->vertex[i*3+2] + length;
        normals[vidx*3  ] = circ->normals[i*3  ];
        normals[vidx*3+1] = circ->normals[i*3+1];
        normals[vidx*3+2] = circ->normals[i*3+2];
        vidx++;
    }
    for(int i = 0; i < circ->nb_line; i++){
        lines[lidx*2  ] = circ->index[i*2  ]+off;
        lines[lidx*2+1] = circ->index[i*2+1]+off;
        lidx++;
    }
    delete circ;
    assert(nb_vert == vidx);
    assert(nb_line == lidx);

#ifndef NDEBUG
    for(int i = 0; i < nb_line*2; i++){
        assert(cyl_cage->index[i] < (int)nb_vert);
    }
#endif
    return cyl_cage;
}

}// END GEN_MESH NAMESPACE =====================================================

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
#include "off_loader.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

// =============================================================================
namespace Off_loader {
// =============================================================================

static void read_vertices(std::ifstream& file,
                          int nb_verts,
                          std::vector<Loader::Vertex>& verts )
{
    verts.resize( nb_verts );
    for(int i = 0; i < nb_verts;i++)
    {
        float a, b, c;
        file >> a >> b >> c;
        verts[i].x = a; verts[i].y = b; verts[i].z = c;
    }
}

// -----------------------------------------------------------------------------

static void add_tri(std::vector<Loader::Tri_face>& tris, int a, int b, int c)
{
    tris.push_back( Loader::Tri_face() );
    const int last = tris.size()-1;
    tris[ last ].v[0] = a;
    tris[ last ].v[1] = b;
    tris[ last ].v[2] = c;
}

// -----------------------------------------------------------------------------

static void triangulate(int face_edges,
                        const std::vector<int>& poly,
                        std::vector<Loader::Tri_face>& tris)
{
    int a = 0;
    int b = 1;
    int c = face_edges - 1;
    int d = face_edges - 2;

    for(int i = 0; i < face_edges - 2; i += 2)
    {
        int v0 = poly[a], v1 = poly[b];
        int v2 = poly[c], v3 = poly[d];
        add_tri(tris, v0, v1, v2);

        if(i < face_edges - 3)
            add_tri(tris, v3, v2, v1);

        a++; b++;
        c--; d--;
    }
}

// -----------------------------------------------------------------------------

static void read_faces(std::ifstream& file,
                       int nb_faces,
                       std::vector<Loader::Tri_face>& tris,
                       std::vector<Loader::Quad_face>& quads )
{
    tris. reserve(nb_faces);
    quads.reserve(nb_faces);

    int max_edges_per_face = 8;
    std::vector<int> poly(max_edges_per_face);
    for(int i = 0; i < nb_faces;i++)
    {
        int face_edges;
        file >> face_edges;
        // Fill triangle
        if(face_edges == 3)
        {
            tris.push_back( Loader::Tri_face() );
            for(int j = 0; j < 3; j++){
                int idx;
                file >> idx;
                tris[tris.size()-1].v[j] = idx;
            }
        }
        //Fill Quad
        else if(face_edges == 4)
        {
            quads.push_back( Loader::Quad_face() );
            for(int j = 0; j < 4; j++){
                int idx;
                file >> idx;
                quads[quads.size()-1].v[j] = idx;
            }
        }
        // Triangulate large faces
        else if(face_edges > 4)
        {
            if(face_edges > max_edges_per_face){
                max_edges_per_face = face_edges;
                poly.resize(max_edges_per_face);
            }

            for(int i = 0; i < face_edges; i++)
                file >> poly[i];

            // add the face to the tris list
            triangulate(face_edges, poly, tris);
        }
    }
}

// -----------------------------------------------------------------------------

bool Off_file::import_file(const std::string& file_path)
{
    _mesh.clear();
    using namespace std;
    int nil;
    ifstream file(file_path.c_str());
    int nb_faces = -1;
    int nb_verts = -1;

    if(!file.is_open()){
        cout << "error loading file : " << file_path << endl;
        return false;
    }

    string line;
    file >> line;
    if(line.compare("OFF\n") == 0){
        cerr << "this is not an OFF file\n";
        return false;
    }

    file >> nb_verts >> nb_faces >> nil;
    read_vertices(file, nb_verts, _mesh._vertices);
    read_faces(file, nb_faces, _mesh._render_faces._tris, _mesh._render_faces._quads);
    file.close();

    int nb_tris  = _mesh._render_faces._tris. size();
    int nb_quads = _mesh._render_faces._quads.size();
    _mesh._triangles.resize( nb_tris + nb_quads * 2);

    for (int i = 0; i < nb_tris; ++i)
        _mesh._triangles[i] = _mesh._render_faces._tris[i];

    for(int i = 0; i < nb_quads; ++i)
    {
        int idx = nb_tris + i*2;

        Loader::Tri_face f0, f1;
        Loader::Quad_face f = _mesh._render_faces._quads[i];
        f.triangulate(f0, f1);

        _mesh._triangles[idx    ] = f0;
        _mesh._triangles[idx + 1] = f1;
    }

    // Push everything in the same group/material group
    int s = _mesh._triangles.size();
    _mesh._groups.push_back( Loader::Group("", 0, s) );

    return true;
}

// -----------------------------------------------------------------------------

bool Off_file::export_file(const std::string& file_path){
    return false;
}

}// END Off_loader =============================================================

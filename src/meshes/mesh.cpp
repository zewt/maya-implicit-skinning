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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <limits>
#include <deque>
#include <map>

#include "glsave.hpp"
#include "glassert.h"
#include "macros.hpp"
#include "mesh.hpp"
#include "loader_mesh.hpp"
#include "timer.hpp"
#include "std_utils.hpp"

// -----------------------------------------------------------------------------

Mesh::Mesh() :
    _is_initialized(false),
    _is_closed(true),
    _has_tex_coords(false),
    _has_normals(false),
    _is_manifold(true),
    _offset(0.f,0.f,0.f),
    _scale(1.f),
    _nb_vert(0),
    _nb_tri(0),
    _nb_quad(0),
    _nb_edges(0),
    _vert(0),
    _is_connected(0),
    _is_side(0),
    _tri(0),
    _quad(0),
    _piv(0),
    _edge_list(0),
    _edge_list_offsets(0),
    _normals(0),
    _tangents(0),
    _tex_coords(0),
    _size_unpacked_vert_array(-1),
    _unpacked_tri(0),
    _unpacked_quad(0),
    _packed_vert_map(0),
    _vbo(GL_ARRAY_BUFFER),
    _normals_bo(GL_ARRAY_BUFFER),
    _tangents_bo(GL_ARRAY_BUFFER),
    _color_bo(GL_ARRAY_BUFFER),
    _tex_bo(GL_ARRAY_BUFFER),
    _point_color_bo(GL_ARRAY_BUFFER),
    _index_bo_tri(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_quad(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_point(GL_ELEMENT_ARRAY_BUFFER)
{
}

// -----------------------------------------------------------------------------

Mesh::Mesh(const std::vector<int>& tri, const std::vector<float>& vert) :
    _is_initialized(true),
    _is_closed(true),
    _has_tex_coords(false),
    _has_normals(false),
    _is_manifold(true),
    _offset(0.f,0.f,0.f),
    _scale(1.f),
    _nb_vert( vert.size() / 3),
    _nb_tri( tri.size() / 3),
    _nb_quad(0),
    _nb_edges(0),
    _vert(0),
    _is_connected(0),
    _is_side(0),
    _tri(0),
    _quad(0),
    _piv(0),
    _edge_list(0),
    _edge_list_offsets(0),
    _normals(0),
    _tangents(0),
    _tex_coords(0),
    _size_unpacked_vert_array( vert.size() / 3),
    _unpacked_tri(0),
    _unpacked_quad(0),
    _packed_vert_map(0),
    _vbo(GL_ARRAY_BUFFER),
    _normals_bo(GL_ARRAY_BUFFER),
    _tangents_bo(GL_ARRAY_BUFFER),
    _color_bo(GL_ARRAY_BUFFER),
    _tex_bo(GL_ARRAY_BUFFER),
    _point_color_bo(GL_ARRAY_BUFFER),
    _index_bo_tri(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_quad(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_point(GL_ELEMENT_ARRAY_BUFFER)
{

    if(_nb_vert > 0)
    {
        _vert            = new float[ _nb_vert * 3 ];
        _is_connected    = new bool [ _nb_vert     ];
        _is_side         = new bool [ _nb_vert     ];
        _packed_vert_map = new Packed_data[_nb_vert];
    }

    for(int i = 0; i < _nb_vert; i++)
    {
        Packed_data d = {i, 1};
        _packed_vert_map[i] = d;

        _vert[i*3  ] = vert[i*3  ];
        _vert[i*3+1] = vert[i*3+1];
        _vert[i*3+2] = vert[i*3+2];

        _is_side     [i] = false;
        _is_connected[i] = false;
    }

    // Copy to packed and unpacked arrays :
    if( _nb_tri > 0){
        _tri           = new int   [_nb_tri  * 3 ];
        _unpacked_tri  = new int   [_nb_tri  * 3 ];
        for(int i = 0; i < _nb_tri*3; i++)
        {
            int idx = _tri[i] = _unpacked_tri[i] = tri[i];
            _is_connected[idx] = true;
        }
    }


    init_vbos();
    compute_piv();
    compute_face_index();
    compute_edges();
    compute_normals();
}

// -----------------------------------------------------------------------------

Mesh::Mesh(const Mesh& m) :
    _is_initialized(m._is_initialized),
    _is_closed(m._is_closed),
    _has_tex_coords(m._has_tex_coords),
    _has_normals(m._has_normals),
    _is_manifold(m._is_manifold),
    _offset(m._offset),
    _scale(m._scale),
    _max_faces_per_vertex(m._max_faces_per_vertex),
    _nb_vert(m._nb_vert),
    _nb_tri(m._nb_tri),
    _nb_quad(m._nb_quad),
    _nb_edges(m._nb_edges),
    _tri_list_per_vert(m._tri_list_per_vert),
    _quad_list_per_vert(m._quad_list_per_vert),
    _size_unpacked_vert_array(m._size_unpacked_vert_array),
    _unpacked_tri_list_per_vert(m._unpacked_tri_list_per_vert),
    _unpacked_quad_list_per_vert(m._unpacked_quad_list_per_vert),

    _vbo(GL_ARRAY_BUFFER),
    _normals_bo(GL_ARRAY_BUFFER),
    _tangents_bo(GL_ARRAY_BUFFER),
    _color_bo(GL_ARRAY_BUFFER),
    _tex_bo(GL_ARRAY_BUFFER),
    _point_color_bo(GL_ARRAY_BUFFER),
    _index_bo_tri(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_quad(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_point(GL_ELEMENT_ARRAY_BUFFER)
{

    _vert            = new float[ _nb_vert*3 ];
    _is_connected    = new bool [ _nb_vert   ];
    _is_side         = new bool [ _nb_vert*3 ];
    _packed_vert_map = new Packed_data[_nb_vert];

    _tri       = new int[3 * _nb_tri             ];
    _quad      = new int[4 * _nb_quad            ];
    _piv       = new int[4 * (_nb_tri + _nb_quad)];
    _edge_list = new int[_nb_edges               ];

    _edge_list_offsets = new int[2*_nb_vert];

    _normals    = _has_normals     ? new float [_size_unpacked_vert_array * 3] : 0;
    _tangents   = m._tangents != 0 ? new float [_size_unpacked_vert_array * 3] : 0;
    _tex_coords = _has_tex_coords  ? new float [_size_unpacked_vert_array * 2] : 0;

    _unpacked_tri  = new int[_nb_tri  * 3];
    _unpacked_quad = new int[_nb_quad * 4];

    for(int i = 0; i < _nb_vert; i++)
    {
        _vert[i*3  ] = m._vert[i*3  ];
        _vert[i*3+1] = m._vert[i*3+1];
        _vert[i*3+2] = m._vert[i*3+2];

        _is_side[i*3  ] = m._is_side[i*3  ];
        _is_side[i*3+1] = m._is_side[i*3+1];
        _is_side[i*3+2] = m._is_side[i*3+2];

        _edge_list_offsets[i*2  ] = m._edge_list_offsets[i*2  ];
        _edge_list_offsets[i*2+1] = m._edge_list_offsets[i*2+1];

        _is_connected   [i] = m._is_connected   [i];
        _packed_vert_map[i] = m._packed_vert_map[i];
    }

    for(int i = 0; i < _nb_tri*3; i++)
    {
        _tri[i] = m._tri[i];
        _unpacked_tri[i] = m._unpacked_tri[i];
    }

    for(int i = 0; i < _nb_quad*4; i++)
    {
        _quad[i] = m._quad[i];
        _unpacked_quad[i] = m._unpacked_quad[i];
    }

    for(int i = 0; i < 4*(_nb_tri + _nb_quad); i++)
        _piv[i] = m._piv[i];

    for(int i = 0; i < _nb_edges; i++)
        _edge_list[i] = m._edge_list[i];

    if( m._has_normals || m._has_tex_coords || m._tangents != 0)
        for(int i = 0; i < _size_unpacked_vert_array; i++)
        {
            if(_has_normals){
                _normals[i*3  ] = m._normals[i*3  ];
                _normals[i*3+1] = m._normals[i*3+1];
                _normals[i*3+2] = m._normals[i*3+2];
            }

            if(_tangents != 0){
                _tangents[i*3  ] = m._tangents[i*3  ];
                _tangents[i*3+1] = m._tangents[i*3+1];
                _tangents[i*3+2] = m._tangents[i*3+2];
            }

            if(_has_tex_coords){
                _tex_coords[i*2  ] = m._tex_coords[i*2  ];
                _tex_coords[i*2+1] = m._tex_coords[i*2+1];
            }
        }

    init_vbos();
}

// -----------------------------------------------------------------------------

Mesh::Mesh(const char* filename) :
    _is_initialized(false),
    _is_closed(true),
    _has_tex_coords(false),
    _has_normals(false),
    _is_manifold(true),
    _offset(0.f,0.f,0.f),
    _scale(1.f),
    _nb_vert(0),
    _nb_tri(0),
    _nb_quad(0),
    _nb_edges(0),
    _vert(0),
    _is_connected(0),
    _is_side(0),
    _tri(0),
    _quad(0),
    _piv(0),
    _edge_list(0),
    _edge_list_offsets(0),
    _normals(0),
    _tangents(0),
    _tex_coords(0),
    _size_unpacked_vert_array(-1),
    _unpacked_tri(0),
    _unpacked_quad(0),
    _packed_vert_map(0),
    _vbo(GL_ARRAY_BUFFER),
    _normals_bo(GL_ARRAY_BUFFER),
    _tangents_bo(GL_ARRAY_BUFFER),
    _color_bo(GL_ARRAY_BUFFER),
    _tex_bo(GL_ARRAY_BUFFER),
    _point_color_bo(GL_ARRAY_BUFFER),
    _index_bo_tri(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_quad(GL_ELEMENT_ARRAY_BUFFER),
    _index_bo_point(GL_ELEMENT_ARRAY_BUFFER)
{
    using namespace std;
    int nil;
    ifstream file(filename);
    int nb_faces;

    if(!file.is_open()){
        cout << "error loading file : " << filename << endl;
        exit(1);
    }
    string line;
    file >> line;
    if(line.compare("OFF\n") == 0){
        cerr << "this is not an OFF file\n";
        return;
    }

    file >> _nb_vert >> nb_faces >> nil;
    _size_unpacked_vert_array = _nb_vert;

    _is_connected    = new bool[_nb_vert];
    _vert            = new float[_nb_vert * 3];
    _packed_vert_map = new Packed_data[_nb_vert];

    std::vector<int> tri_index(nb_faces * 3 * 2);
    std::vector<int> quad_index(nb_faces * 4);

    for(int i = 0; i < _nb_vert;i++)
    {
        float a,b,c;
        file >> a >> b >> c;
        _vert[3*i] = a; _vert[3*i + 1] = b; _vert[3*i + 2] = c;
        _is_connected[i] = false;
        Packed_data d = {i, 1};
        _packed_vert_map[i] = d;
    }

    _nb_tri = _nb_quad = 0;
    int k = 0, k_quad = 0, max_edges_per_face = 8;
    for(int i = 0; i < nb_faces;i++)
    {
        int face_edges;
        file >> face_edges;
        if(face_edges == 3)
        {
            for(int j = 0; j < 3; j++){
                int idx;
                file >> idx;
                tri_index[k++] = idx;
                _is_connected[idx] = true;
            }
            _nb_tri++;
        }
        if(face_edges == 4)
        {
            for(int j = 0; j < 4; j++){
                int idx;
                file >> idx;
                quad_index[k_quad++] =idx;
                _is_connected[idx] = true;
            }
            _nb_quad++;
        }
        if(face_edges > 4)
        {
            int* v_face = new int[max_edges_per_face];
            cout << "large face: " << face_edges << "at " << _nb_tri << "\n";
            if(face_edges > max_edges_per_face){
                delete[] v_face;
                max_edges_per_face = face_edges;
                v_face = new int[max_edges_per_face];
            }
            for(int i = 0; i < face_edges; i++){
                file >> v_face[i];
            }
            int a = 0;
            int b = 1;
            int c = face_edges - 1;
            int d = face_edges - 2;
            for(int i = 0; i < face_edges - 2; i += 2){
                int v0 = v_face[a], v1 = v_face[b];
                int v2 = v_face[c], v3 = v_face[d];
                tri_index[k++] = v0; tri_index[k++] = v1; tri_index[k++] = v2;
                _nb_tri++;
                if(i < face_edges - 3)
                {
                    tri_index[k++] = v3; tri_index[k++] = v2; tri_index[k++] = v1;
                    _nb_tri++;
                }
                a++; b++;
                c--; d--;
            }
            delete[] v_face;
        }
    }
    file.close();

    // Copy to packed and unpacked arrays :
    if( _nb_tri > 0){
        _tri           = new int   [_nb_tri  * 3 ];
        _unpacked_tri  = new int   [_nb_tri  * 3 ];
        for(int i = 0; i < _nb_tri*3; i++)
            _tri[i] = _unpacked_tri[i] = tri_index[i];
    }

    if( _nb_quad > 0){
        _quad          = new int   [_nb_quad * 4 ];
        _unpacked_quad = new int   [_nb_quad * 4 ];
        for(int i = 0; i < _nb_quad*4; i++)
            _quad[i] = _unpacked_quad[i] = quad_index[i];
    }

    compute_piv();
    compute_face_index();
    compute_edges();
    compute_normals();
    init_vbos();

    _is_initialized = true;
}

// -----------------------------------------------------------------------------

Mesh::~Mesh(){
    free_mesh_data();
}

// -----------------------------------------------------------------------------

void Mesh::free_mesh_data()
{
    _is_initialized = false;
    delete[] _is_connected;
    delete[] _vert;
    delete[] _tri;
    delete[] _quad;
    delete[] _normals;
    delete[] _tangents;
    delete[] _tex_coords;
    delete[] _packed_vert_map;
    delete[] _unpacked_tri;
    delete[] _unpacked_quad;
    delete[] _piv;
    delete[] _edge_list;
    delete[] _edge_list_offsets;
    _is_connected      = 0;
    _vert              = 0;
    _tri               = 0;
    _quad              = 0;
    _normals           = 0;
    _tangents          = 0;
    _tex_coords        = 0;
    _packed_vert_map   = 0;
    _unpacked_tri      = 0;
    _unpacked_quad     = 0;
    _piv               = 0;
    _edge_list         = 0;
    _edge_list_offsets = 0;

    _tri_list_per_vert.clear();
    _quad_list_per_vert.clear();
    _unpacked_tri_list_per_vert.clear();
    _unpacked_quad_list_per_vert.clear();

    //////////////// Register buffer
    if(_nb_vert > 0){
        _vbo.cuda_unregister();
        _normals_bo.cuda_unregister();
        _color_bo.cuda_unregister();

        if(_nb_tri  > 0) _index_bo_tri. cuda_unregister();
        if(_nb_quad > 0) _index_bo_quad.cuda_unregister();

        if(_has_tex_coords)
        {
            _tex_bo.cuda_unregister();
            _tangents_bo.cuda_unregister();
        }
    }
    //////////////// End Register buffer
}

// -----------------------------------------------------------------------------

void Mesh::export_off(const char* filename, bool invert_index) const
{
    using namespace std;
    ofstream file(filename);
    if( !file.is_open() ){
        cerr << "Error creating file: " << filename << endl;
        exit(1);
    }
    file << "OFF" << endl;
    file << _nb_vert << ' ' << get_nb_faces() << " 0" << endl;

    for(int i = 0; i < _nb_vert; i++){
        file << (_vert[3 * i    ] * 1.f / _scale - _offset.x) << ' '
             << (_vert[3 * i + 1] * 1.f / _scale - _offset.y) << ' '
             << (_vert[3 * i + 2] * 1.f / _scale - _offset.z) << ' ' << endl;
    }

    for(int i = 0; i < _nb_tri; i++) {
        int a = _tri[3*i  ];
        int b = _tri[3*i+1];
        int c = _tri[3*i+2];
        if(invert_index)
            file << "3 " << c << ' ' << b << ' ' << a << endl;
        else
            file << "3 " << a << ' ' << b << ' ' << c << endl;
    }

    for(int i = 0; i < _nb_quad; i++) {
        int a = _quad[4*i  ];
        int b = _quad[4*i+1];
        int c = _quad[4*i+2];
        int d = _quad[4*i+3];
        if(invert_index)
            file << "4 " << d << ' ' << c << ' ' << b << ' ' << a << endl;
        else
            file << "4 " << a << ' ' << b << ' ' << c << ' ' << d << endl;
    }
}

// -----------------------------------------------------------------------------

void Mesh::compute_piv()
{
    delete[] _piv;
    _piv = new int [4 * (_nb_tri + _nb_quad)];
    int* pic = new int [_nb_vert]; // Nb faces per vertices
    for(int i = 0; i < _nb_vert; i++){
        pic[i] = 0;
    }
    int imax = 0; // Max number of faces for every vertices
    for(int i = 0; i < _nb_tri; i++){
        int ia = _tri[3*i    ];
        int ib = _tri[3*i + 1];
        int ic = _tri[3*i + 2];

        _piv[4*i] = pic[ia]++;
        if(imax < pic[ia]) imax = pic[ia];
        _piv[4*i+1] = pic[ib]++;
        if(imax < pic[ib]) imax = pic[ib];
        _piv[4*i+2] = pic[ic]++;
        if(imax < pic[ic]) imax = pic[ic];
        _piv[4*i+3] = 0;
    }

    for(int i = 0; i < _nb_quad; i++){
        int j = i + _nb_tri;
        int ia = _quad[4*i    ];
        int ib = _quad[4*i + 1];
        int ic = _quad[4*i + 2];
        int id = _quad[4*i + 3];

        _piv[4*j] = pic[ia]++;
        if(imax < pic[ia]) imax = pic[ia];
        _piv[4*j+1] = pic[ib]++;
        if(imax < pic[ib]) imax = pic[ib];
        _piv[4*j+2] = pic[ic]++;
        if(imax < pic[ic]) imax = pic[ic];
        _piv[4*j+3] = pic[id]++;
        if(imax < pic[id]) imax = pic[id];
    }
    _max_faces_per_vertex = imax;
    delete [] pic;
}

// -----------------------------------------------------------------------------

void Mesh::center_and_resize(float max_size)
{
    float xmin, ymin, zmin;
    float xmax, ymax, zmax;
    xmin = ymin = zmin =  std::numeric_limits<float>::infinity();
    xmax = ymax = zmax = -std::numeric_limits<float>::infinity();

    for(int i = 0; i < _nb_vert; i++){
        float x = _vert[3*i  ];
        float y = _vert[3*i+1];
        float z = _vert[3*i+2];
        //printf("%f %f %f\n",x,y,z);
        xmin = (x < xmin)? x : xmin;
        xmax = (x > xmax)? x : xmax;
        ymin = (y < ymin)? y : ymin;
        ymax = (y > ymax)? y : ymax;
        zmin = (z < zmin)? z : zmin;
        zmax = (z > zmax)? z : zmax;
    }
    float dx = xmax - xmin;
    float dy = ymax - ymin;
    float dz = zmax - zmin;
    float du = (dx > dy)?((dx > dz)?dx:dz):((dy>dz)?dy:dz);
    float scale_f = max_size / du;
    _scale = scale_f;
    _offset.x = - 0.5f * (xmax + xmin);
    _offset.y = - 0.5f * (ymax + ymin);
    _offset.z = - 0.5f * (zmax + zmin);
    for(int i = 0; i < _nb_vert; i++)
    {
        float& x = _vert[3*i  ];
        float& y = _vert[3*i+1];
        float& z = _vert[3*i+2];
        x = (x + _offset.x) * scale_f;
        y = (y + _offset.y) * scale_f;
        z = (z + _offset.z) * scale_f;
    }

    // update the vbo
    update_unpacked_vert();
}

// -----------------------------------------------------------------------------

void Mesh::compute_normals()
{
    delete[] _normals;
    float* new_normals = new float[_nb_vert * 3];

    _has_normals = true;
    for(int i = 0; i < 3 * _nb_vert; i++){
        new_normals[i] = 0.0f;
    }
    for(int i = 0; i < _nb_tri; i++){
        int va = _tri[3*i  ];
        int vb = _tri[3*i+1];
        int vc = _tri[3*i+2];
        float xa = _vert[3*va], ya = _vert[3*va+1], za = _vert[3*va+2];
        float xb = _vert[3*vb], yb = _vert[3*vb+1], zb = _vert[3*vb+2];
        float xc = _vert[3*vc], yc = _vert[3*vc+1], zc = _vert[3*vc+2];
        float x0 = xb - xa, y0 = yb - ya, z0 = zb - za;
        float x1 = xc - xa, y1 = yc - ya, z1 = zc - za;
        float nx = y0 * z1 - z0 * y1;
        float ny = z0 * x1 - x0 * z1;
        float nz = x0 * y1 - y0 * x1;
        float norm = -sqrtf(nx * nx + ny * ny + nz * nz);
        nx /= norm; ny /= norm, nz /= norm;
        float fnx =  nx, fny = ny, fnz = nz;
        new_normals[3 * va] += fnx;	new_normals[3 * va + 1] += fny;	new_normals[3 * va + 2] += fnz;
        new_normals[3 * vb] += fnx;	new_normals[3 * vb + 1] += fny;	new_normals[3 * vb + 2] += fnz;
        new_normals[3 * vc] += fnx;	new_normals[3 * vc + 1] += fny;	new_normals[3 * vc + 2] += fnz;
    }

    for(int i = 0; i < _nb_quad; i++){
        int va = _quad[4*i  ];
        int vb = _quad[4*i+1];
        int vc = _quad[4*i+2];
        int vd = _quad[4*i+3];
        float xa = _vert[3*va], ya = _vert[3*va+1], za = _vert[3*va+2];
        float xb = _vert[3*vb], yb = _vert[3*vb+1], zb = _vert[3*vb+2];
        float xc = _vert[3*vc], yc = _vert[3*vc+1], zc = _vert[3*vc+2];
        float xd = _vert[3*vd], yd = _vert[3*vd+1], zd = _vert[3*vd+2];
        float x0 = xb - xa + xc - xd, y0 = yb - ya + yc - yd, z0 = zb - za + zc - zd;
        float x1 = xd - xa + xc - xb, y1 = yd - ya + yc - yb, z1 = zd - za + zc - zb;
        float nx = y0 * z1 - z0 * y1;
        float ny = z0 * x1 - x0 * z1;
        float nz = x0 * y1 - y0 * x1;
        float norm = -sqrtf(nx * nx + ny * ny + nz * nz);
        nx /= norm; ny /= norm, nz /= norm;
        new_normals[3 * va] += nx; new_normals[3 * va + 1] += ny; new_normals[3 * va + 2] += nz;
        new_normals[3 * vb] += nx; new_normals[3 * vb + 1] += ny; new_normals[3 * vb + 2] += nz;
        new_normals[3 * vc] += nx; new_normals[3 * vc + 1] += ny; new_normals[3 * vc + 2] += nz;
        new_normals[3 * vd] += nx; new_normals[3 * vd + 1] += ny; new_normals[3 * vd + 2] += nz;
    }

    std::cout << "normals :\n";
    for(int i = 0; i < _nb_vert; i++){
        float& nx = new_normals[3 * i    ];
        float& ny = new_normals[3 * i + 1];
        float& nz = new_normals[3 * i + 2];
        float norm = sqrtf(nx * nx + ny * ny + nz * nz);
        if(norm > 0.f){
            nx /= -norm; ny /= -norm, nz /= -norm;
        }
    }

    // unpack the normals we've just calculated in new_normals
    int n_size = _size_unpacked_vert_array * 3;
    _normals = new float [n_size];

    for(int i = 0; i < _nb_vert; i++)
    {
        const Packed_data d = _packed_vert_map[i];
        for(int j = 0; j < d.nb_ocurrence; j++)
        {
            const int p_idx = d.idx_data_unpacked + j;
            _normals[p_idx*3    ] = new_normals[i*3    ];
            _normals[p_idx*3 + 1] = new_normals[i*3 + 1];
            _normals[p_idx*3 + 2] = new_normals[i*3 + 2];
        }
    }
     _normals_bo.set_data(n_size, _normals, GL_STATIC_DRAW);
    delete[] new_normals;
}

// -----------------------------------------------------------------------------

void Mesh::compute_tangents()
{
    delete[] _tangents;
    float* new_tangents = new float[_nb_vert * 3];

    _has_normals = true;
    for(int i = 0; i < 3 * _nb_vert; i++){
        new_tangents[i] = 0.0f;
    }

    for(int i = 0; i < _nb_tri; i++)
    {
        Tri_idx idx  = *(Tri_idx*)(_tri+i*3);
        Tri_idx uidx = *(Tri_idx*)(_unpacked_tri+i*3);

        Vec3_cu va = *(Vec3_cu*)(_vert+idx.a*3);
        Vec3_cu vb = *(Vec3_cu*)(_vert+idx.b*3);
        Vec3_cu vc = *(Vec3_cu*)(_vert+idx.c*3);

        Tex_coords ta = *(Tex_coords*)(_tex_coords+uidx.a*2);
        Tex_coords tb = *(Tex_coords*)(_tex_coords+uidx.b*2);
        Tex_coords tc = *(Tex_coords*)(_tex_coords+uidx.c*2);

        Vec3_cu v1 = Vec3_cu(vb.x - va.x, vb.y - va.y, vb.z - va.z);
        Vec3_cu v2 = Vec3_cu(vc.x - va.x, vc.y - va.y, vc.z - va.z);

        Tex_coords st1 = Tex_coords(tb.u - ta.u, tb.v - ta.v);
        Tex_coords st2 = Tex_coords(tc.u - ta.u, tc.v - ta.v);

        float coef = 1.f / (st1.u * st2.v - st2.u * st1.v);
        float tangent[3];
        tangent[0] = coef * ((v1.x * st2.v)  + (v2.x * -st1.v));
        tangent[1] = coef * ((v1.y * st2.v)  + (v2.y * -st1.v));
        tangent[2] = coef * ((v1.z * st2.v)  + (v2.z * -st1.v));

        for(int j = 0; j < 3; j++)
        {
            new_tangents[idx.a*3 + j] += tangent[j];
            new_tangents[idx.b*3 + j] += tangent[j];
            new_tangents[idx.c*3 + j] += tangent[j];
        }
    }

    for(int i = 0; i < _nb_quad; i++){
        // TODO: check this code with a 'bump mapped' quad mesh
        Quad_idx idx  = *(Quad_idx*)(_quad+i*4);
        Quad_idx uidx = *(Quad_idx*)(_unpacked_quad+i*4);

        Vec3_cu va = *(Vec3_cu*)(_vert+idx.a*3);
        Vec3_cu vb = *(Vec3_cu*)(_vert+idx.b*3);
        Vec3_cu vc = *(Vec3_cu*)(_vert+idx.c*3);
        Vec3_cu vd = *(Vec3_cu*)(_vert+idx.d*3);

        Tex_coords ta = *(Tex_coords*)(_tex_coords+uidx.a*2);
        Tex_coords tb = *(Tex_coords*)(_tex_coords+uidx.b*2);
        Tex_coords tc = *(Tex_coords*)(_tex_coords+uidx.c*2);
        Tex_coords td = *(Tex_coords*)(_tex_coords+uidx.d*2);

        // Tangent of the triangle abc
        Vec3_cu v1 = Vec3_cu(vb.x - va.x, vb.y - va.y, vb.z - va.z);
        Vec3_cu v2 = Vec3_cu(vc.x - va.x, vc.y - va.y, vc.z - va.z);

        Tex_coords st1 = Tex_coords(tb.u - ta.u, tb.v - ta.v);
        Tex_coords st2 = Tex_coords(tc.u - ta.u, tc.v - ta.v);

        float coef = 1.f / (st1.u * st2.v - st2.u * st1.v);
        float tangent_0[3];
        tangent_0[0] = coef * ((v1.x * st2.v)  + (v2.x * -st1.v));
        tangent_0[1] = coef * ((v1.y * st2.v)  + (v2.y * -st1.v));
        tangent_0[2] = coef * ((v1.z * st2.v)  + (v2.z * -st1.v));

        // Tangent of the triangle acd
        v1 = Vec3_cu(va.x - vc.x, va.y - vc.y, va.z - vc.z);
        v2 = Vec3_cu(vd.x - vc.x, vd.y - vc.y, vd.z - vc.z);

        st1 = Tex_coords(ta.u - tc.u, ta.v - tc.v);
        st2 = Tex_coords(td.u - tc.u, td.v - tc.v);

        coef = 1.f / (st1.u * st2.v - st2.u * st1.v);
        float tangent_1[3];
        tangent_1[0] = coef * ((v1.x * st2.v)  + (v2.x * -st1.v));
        tangent_1[1] = coef * ((v1.y * st2.v)  + (v2.y * -st1.v));
        tangent_1[2] = coef * ((v1.z * st2.v)  + (v2.z * -st1.v));

        for(int j = 0; j < 3; j++)
        {
            new_tangents[idx.a*3 + j] += tangent_0[j];
            new_tangents[idx.b*3 + j] += tangent_0[j];
            new_tangents[idx.c*3 + j] += tangent_0[j];

            new_tangents[idx.a*3 + j] += tangent_1[j];
            new_tangents[idx.c*3 + j] += tangent_1[j];
            new_tangents[idx.d*3 + j] += tangent_1[j];
        }
    }

    // unpack the tangents we've just calculated in new_tangents
    int n_size = _size_unpacked_vert_array * 3;
    _tangents = new float [n_size];

    for(int i = 0; i < _nb_vert; i++)
    {
        const Packed_data d = _packed_vert_map[i];
        for(int j = 0; j < d.nb_ocurrence; j++)
        {
            const int p_idx = d.idx_data_unpacked + j;
            float tx = new_tangents[i*3    ];
            float ty = new_tangents[i*3 + 1];
            float tz = new_tangents[i*3 + 2];
            float norm = sqrtf(tx * tx + ty * ty + tz * tz);

            _tangents[p_idx*3    ] = tx / norm;
            _tangents[p_idx*3 + 1] = ty / norm;
            _tangents[p_idx*3 + 2] = tz / norm;
        }
    }
    _tangents_bo.set_data(n_size, _tangents, GL_STATIC_DRAW);
    delete[] new_tangents;
}

// -----------------------------------------------------------------------------

void Mesh::compute_face_index()
{
    assert(_size_unpacked_vert_array > 0);

    _unpacked_tri_list_per_vert.clear();
    _unpacked_quad_list_per_vert.clear();
    _tri_list_per_vert.clear();
    _quad_list_per_vert.clear();
    _tri_list_per_vert. resize(_nb_vert);
    _quad_list_per_vert.resize(_nb_vert);
    _unpacked_tri_list_per_vert.resize(_size_unpacked_vert_array);
    _unpacked_quad_list_per_vert.resize(_size_unpacked_vert_array);

    for(int i = 0; i < _nb_tri; i++){
        for(int j = 0; j < 3; j++){
            int v = _tri[3*i +j ];
            assert(v>=0);
            _tri_list_per_vert[v].push_back(i);
            v = _unpacked_tri[3*i + j ];
            _unpacked_tri_list_per_vert[v].push_back(i);
        }
    }

    for(int i = 0; i < _nb_quad; i++){
        for(int j = 0; j < 4; j++){
            int v = _quad[4*i + j];
            _quad_list_per_vert[v].push_back(i);
            v = _unpacked_quad[4*i + j];
            _unpacked_quad_list_per_vert[v].push_back(i);
        }
    }
}

// -----------------------------------------------------------------------------

void Mesh::save(Loader::Abs_mesh& mesh)
{
    // FIXME: handling quads:

    // Copy vertices array
    mesh._vertices.clear();
    mesh._vertices.resize( _nb_vert );
    for(unsigned i = 0; i < mesh._vertices.size(); ++i){
        Vec3_cu v(_vert[i*3], _vert[i*3+1], _vert[i*3+2]);
        mesh._vertices[i] = Loader::Vertex(v.x, v.y, v.z);
    }

    // Copy normals array
    mesh._normals.clear();
    mesh._normals.resize( _size_unpacked_vert_array );
    for(unsigned i = 0; i < mesh._normals.size(); ++i){
        Vec3_cu v(_normals[i*3], _normals[i*3+1], _normals[i*3+2]);
        mesh._normals[i] = Loader::Normal(v.x, v.y, v.z);
    }

    // Copy texture coordinates array
    mesh._texCoords.clear();
    mesh._texCoords.resize( _size_unpacked_vert_array );
    for(unsigned i = 0; i < mesh._texCoords.size(); ++i)
        mesh._texCoords[i] = Loader::TexCoord( _tex_coords[i*2], _tex_coords[i*2+1] );

    // Copy face index array
    mesh._triangles.clear();
    mesh._triangles.resize( _nb_tri );
    for(unsigned i = 0; i < mesh._triangles.size(); ++i){
        Loader::Tri_face F;
        // Vertex index
        F.v[0] = _tri[i*3];  F.v[1] = _tri[i*3+1];  F.v[2] = _tri[i*3+2];
        // Normal index
        const int* ptr = _unpacked_tri;
        F.n[0] = ptr[i*3]; F.n[1] = ptr[i*3+1]; F.n[2] = ptr[i*3+2];
        // Tex coords index
        F.t[0] = ptr[i*3]; F.t[1] = ptr[i*3+1]; F.t[2] = ptr[i*3+2];

        mesh._triangles[i] = F;
    }

    // copy groups (we don't actually handle groups so we just one big root
    // group which contains all the materials groups)
    mesh._groups.clear();
    mesh._groups.resize( 1 );

    Loader::Group G;
    G._start_face = 0;
    G._end_face = mesh._triangles.size();
    //G.start_point = 0;
    //G._end_point = 0;
    G._name = "_";

    mesh._groups[0] = G;
}

// -----------------------------------------------------------------------------

void Mesh::load(const Loader::Abs_mesh& mesh, const std::string& mesh_path)
{
    _is_initialized = false;
    free_mesh_data();

    _nb_vert = mesh._vertices.size();
    _nb_tri  = mesh._triangles.size();
    _nb_quad = 0; // For now the loader triangulates every faces

    _vert          = new float [_nb_vert * 3];
    _tri           = new int   [_nb_tri  * 3];
    _quad          = 0;
    _unpacked_tri  = new int   [_nb_tri  * 3];
    _unpacked_quad = 0;

    _is_connected = new bool[_nb_vert];
    _is_side      = new bool[_nb_vert];

    // Copy vertex coordinates
    for( int i = 0; i < _nb_vert; i++)
    {
        (*(Loader::Vertex*)(_vert+i*3)) = mesh._vertices[i];
        _is_connected[i] = false;
    }

    // Build the list of texture coordinates indices and normals indices per
    // vertices indices.
    // Also we copy the packed triangle index in '_tri'
    std::vector<std::map<std::pair<int,int>,int> > pair_per_vert(_nb_vert);
    std::vector<int> nb_pair_per_vert(_nb_vert, 0);
    for( int i = 0; i < _nb_tri; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            // Fill triangle index
            int v_idx = mesh._triangles[i].v[j];
            int n_idx = mesh._triangles[i].n[j];
            int t_idx = mesh._triangles[i].t[j];

            _is_connected[v_idx] = true;
            _tri[i*3+j] = v_idx;

            std::pair<int, int> pair(t_idx, n_idx);
            std::map<std::pair<int, int>,int>& map = pair_per_vert[v_idx];
            if( map.find(pair) == map.end() )
            {
                map[pair] = nb_pair_per_vert[v_idx];
                nb_pair_per_vert[v_idx]++;
            }
        }
    }

    // We now build the mapping between packed vertex coordinates and unpacked
    // vertex coortinates, so that each vertex in the unpacked form has its own
    // texture coordinates and/or normal direction.
    _packed_vert_map = new Packed_data[_nb_vert];
    int off = 0;
    for( int i = 0; i < _nb_vert; i++)
    {
        int nb_elt = std::max(1, nb_pair_per_vert[i]);

        Packed_data tuple;
        tuple.idx_data_unpacked = off;
        tuple.nb_ocurrence      = nb_elt;

        _packed_vert_map[i] = tuple;

        off += nb_elt;
    }


    _size_unpacked_vert_array = off;
    _normals    = new float [_size_unpacked_vert_array * 3];
    _tex_coords = new float [_size_unpacked_vert_array * 2];

    // Copy triangles index, normals and texture coordinates
    _has_tex_coords = false;
    _has_normals    = false;
    for( int i = 0; i < _nb_tri; i++)
    {
        for( int j = 0; j < 3; j++)
        {
            int v_idx = mesh._triangles[i].v[j];
            int n_idx = mesh._triangles[i].n[j];
            int t_idx = mesh._triangles[i].t[j];

            std::pair<int, int> pair(t_idx, n_idx);
            int off = pair_per_vert[v_idx][pair];

            assert(off < _packed_vert_map[v_idx].nb_ocurrence);

            int v_unpacked = _packed_vert_map[v_idx].idx_data_unpacked + off;

            // Fill unpacked triangle index
            _unpacked_tri[i*3+j] = v_unpacked;
            // Fill normal as there index match the unpacked vertex array
            if( n_idx != -1 )
                *((Loader::Normal*)(_normals+v_unpacked*3)) = mesh._normals[n_idx];
            else
                *((Loader::Normal*)(_normals+v_unpacked*3)) = Loader::Normal();

            // Fill texture coordinates as there index match the unpacked vertex array
            if( t_idx != -1 )
                *((Loader::TexCoord*)(_tex_coords+v_unpacked*2)) = mesh._texCoords[t_idx];
            else
                *((Loader::TexCoord*)(_tex_coords+v_unpacked*2)) = Loader::TexCoord();

            _has_tex_coords = _has_tex_coords || (t_idx != -1);
            _has_normals    = _has_normals    || (n_idx != -1);
        }
    }

    if( !_has_normals )
        compute_normals();
    if(_has_tex_coords)
        compute_tangents();
    // Initialize VBOs
    init_vbos();
    compute_piv();
    compute_face_index();
    compute_edges();
    _is_initialized = true;
}

// -----------------------------------------------------------------------------

void Mesh::smoothen_mesh(float smooth_factor,
                         int nb_iter,
                         int nb_min_neighbours)
{
    if(nb_iter == 0) return;

    float* vertices_data     = _vert;
    float* new_vertices_data = new float[3*_nb_vert];
    int*   vert_neig         = new int[_nb_vert];

    for(int k = 0; k < nb_iter; k++)
    {
        for(int i = 0; i < _nb_vert; i++)
        {
            new_vertices_data[3*i  ] = 0.f;
            new_vertices_data[3*i+1] = 0.f;
            new_vertices_data[3*i+2] = 0.f;
            vert_neig[i] = 0;
        }

        for(int i = 0; i < _nb_tri; i++)
        {
            int a = _tri[3*i  ];
            int b = _tri[3*i+1];
            int c = _tri[3*i+2];
            new_vertices_data[3*a  ] += vertices_data[3*b  ] + vertices_data[3*c  ];
            new_vertices_data[3*a+1] += vertices_data[3*b+1] + vertices_data[3*c+1];
            new_vertices_data[3*a+2] += vertices_data[3*b+2] + vertices_data[3*c+2];

            new_vertices_data[3*b  ] += vertices_data[3*a  ] + vertices_data[3*c  ];
            new_vertices_data[3*b+1] += vertices_data[3*a+1] + vertices_data[3*c+1];
            new_vertices_data[3*b+2] += vertices_data[3*a+2] + vertices_data[3*c+2];

            new_vertices_data[3*c  ] += vertices_data[3*b  ] + vertices_data[3*a  ];
            new_vertices_data[3*c+1] += vertices_data[3*b+1] + vertices_data[3*a+1];
            new_vertices_data[3*c+2] += vertices_data[3*b+2] + vertices_data[3*a+2];

            vert_neig[a] += 2;
            vert_neig[b] += 2;
            vert_neig[c] += 2;
        }

        for(int i = 0; i < _nb_quad; i++)
        {
            int a = _quad[4*i  ];
            int b = _quad[4*i+1];
            int c = _quad[4*i+2];
            int d = _quad[4*i+3];
            new_vertices_data[3*a  ] += vertices_data[3*b  ] + vertices_data[3*d  ];
            new_vertices_data[3*a+1] += vertices_data[3*b+1] + vertices_data[3*d+1];
            new_vertices_data[3*a+2] += vertices_data[3*b+2] + vertices_data[3*d+2];

            new_vertices_data[3*b  ] += vertices_data[3*a  ] + vertices_data[3*c  ];
            new_vertices_data[3*b+1] += vertices_data[3*a+1] + vertices_data[3*c+1];
            new_vertices_data[3*b+2] += vertices_data[3*a+2] + vertices_data[3*c+2];

            new_vertices_data[3*c  ] += vertices_data[3*b  ] + vertices_data[3*d  ];
            new_vertices_data[3*c+1] += vertices_data[3*b+1] + vertices_data[3*d+1];
            new_vertices_data[3*c+2] += vertices_data[3*b+2] + vertices_data[3*d+2];

            new_vertices_data[3*d  ] += vertices_data[3*a  ] + vertices_data[3*c  ];
            new_vertices_data[3*d+1] += vertices_data[3*a+1] + vertices_data[3*c+1];
            new_vertices_data[3*d+2] += vertices_data[3*a+2] + vertices_data[3*c+2];

            vert_neig[a] += 2;
            vert_neig[b] += 2;
            vert_neig[c] += 2;
            vert_neig[d] += 2;
        }

        for(int i = 0; i < _nb_vert; i++)
        {
            if(vert_neig[i] > nb_min_neighbours && !_is_side[i])
            {
                float iv = 1.f / vert_neig[i];
                new_vertices_data[3*i  ] *= iv;
                new_vertices_data[3*i+1] *= iv;
                new_vertices_data[3*i+2] *= iv;
            }
            else
            {
                new_vertices_data[3*i  ] = vertices_data[3*i  ];
                new_vertices_data[3*i+1] = vertices_data[3*i+1];
                new_vertices_data[3*i+2] = vertices_data[3*i+2];
            }

            float u = smooth_factor;
            vertices_data[3*i  ] = vertices_data[3*i  ] * (1.f-u) + new_vertices_data[3*i  ] * u;
            vertices_data[3*i+1] = vertices_data[3*i+1] * (1.f-u) + new_vertices_data[3*i+1] * u;
            vertices_data[3*i+2] = vertices_data[3*i+2] * (1.f-u) + new_vertices_data[3*i+2] * u;
        }


        float* tmp = new_vertices_data;
        new_vertices_data = vertices_data;
        vertices_data = tmp;
    }

    if((nb_iter%2) == 0)
    {
        _vert = vertices_data;
        delete[] new_vertices_data;
    }
    else
    {
        _vert = new_vertices_data;
        delete[] vertices_data;
    }

    delete[] vert_neig;
    // update the vbo
    update_unpacked_vert();
}

// -----------------------------------------------------------------------------

std::pair<int, int> Mesh::pair_from_tri(int index_tri, int current_vert)
{
    int ids[2] = {-1, -1};
    for(int i=0; i<3; i++){
        int vert_id = _tri[index_tri*3 + i];
        if(vert_id == current_vert){
            ids[0] = _tri[index_tri*3 + (i+1)%3];
            ids[1] = _tri[index_tri*3 + (i+2)%3];
            break;
        }
    }

    return std::pair<int, int>(ids[0], ids[1]);
}

// -----------------------------------------------------------------------------

std::pair<int, int> Mesh::pair_from_quad(int index_quad, int current_vert, int n)
{
    assert(n == 1 || n == 0);

    int ids[3] = {-1, -1, -1};
    for(int i=0; i<4; i++){
        int vert_id = _quad[index_quad*4 + i];
        if(vert_id == current_vert){
            ids[0] = _quad[index_quad*4 + (i+1)%4];
            ids[1] = _quad[index_quad*4 + (i+2)%4];
            ids[2] = _quad[index_quad*4 + (i+3)%4];
            break;
        }
    }

    return std::pair<int, int>(ids[0+n], ids[1+n]);
}

// -----------------------------------------------------------------------------

bool add_to_ring(std::deque<int>& ring, std::pair<int, int> p)
{
    if(ring[ring.size()-1] == p.first)
    {
        ring.push_back(p.second);
        return true;
    }
    else if(ring[ring.size()-1] == p.second)
    {

        ring.push_back(p.first);
        return true;
    }
    else if(ring[0] == p.second)
    {
        ring.push_front(p.first);
        return true;
    }
    else if(ring[0] == p.first)
    {
        ring.push_front(p.second);
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------

/// Add an element to the ring only if it does not alredy exists
/// @return true if already exists
bool add_to_ring(std::deque<int>& ring, int neigh)
{
    std::deque<int>::iterator it;
    for(it = ring.begin(); it != ring.end(); ++it)
        if(*it == neigh) return true;

    ring.push_back( neigh );
    return false;
}

// -----------------------------------------------------------------------------

void Mesh::compute_edges()
{
    Timer t;
    t.start();

    _not_manifold_verts.clear();
    _on_side_verts.clear();
    delete[] _is_side;
    _is_side = new bool[_nb_vert];
    for(int i = 0; i < _nb_vert; i++)
        _is_side[i] = false;

    _nb_edges = 0;
    std::vector<std::vector<int> > neighborhood_list(_nb_vert);
    std::vector<std::pair<int, int> > list_pairs;
    list_pairs.reserve(16);
    for(int i = 0; i < _nb_vert; i++)
    {
        // We suppose the faces are quads to reserve memory
        if( _max_faces_per_vertex > 0)
            neighborhood_list.reserve(_max_faces_per_vertex*3);

        if( is_disconnect(i) ) continue; // TODO should be is_side = true no ?

        list_pairs.clear();
        // fill pairs with the first ring of neighborhood of quads and triangles
        for(unsigned int j=0; j<_tri_list_per_vert[i].size(); j++)
            list_pairs.push_back(pair_from_tri(_tri_list_per_vert[i][j], i));

        for(unsigned int j=0; j<_quad_list_per_vert[i].size(); j++)
        {
            list_pairs.push_back(pair_from_quad(_quad_list_per_vert[i][j], i, 0));
            list_pairs.push_back(pair_from_quad(_quad_list_per_vert[i][j], i, 1));
        }

        // Try to build the ordered list of the first ring of neighborhood of i
        std::deque<int> ring;
        ring.push_back(list_pairs[0].first );
        ring.push_back(list_pairs[0].second);
        std::vector<std::pair<int, int> >::iterator it = list_pairs.begin();
        list_pairs.erase(it);
        unsigned int  pairs_left = list_pairs.size();
        bool manifold   = true;
        while( (pairs_left = list_pairs.size()) != 0)
        {
            for(it = list_pairs.begin(); it < list_pairs.end(); ++it)
            {
                if(add_to_ring(ring, *it))
                {
                    list_pairs.erase(it);
                    break;
                }
            }

            if(pairs_left == list_pairs.size())
            {
                // Not manifold we push neighborhoods of vert 'i'
                // in a random order
                add_to_ring(ring, list_pairs[0].first );
                add_to_ring(ring, list_pairs[0].second);
                list_pairs.erase(list_pairs.begin());
                manifold = false;
            }
        }

        if(!manifold)
        {
            std::cerr << "WARNING : The mesh is clearly not 2-manifold !\n";
            std::cerr << "Check vertex index : " << i << std::endl;
            _is_manifold = false;
            _not_manifold_verts.push_back(i);
        }


        if(ring[0] != ring[ring.size()-1]){
            _is_side[i] = true;
            _on_side_verts.push_back(i);
            _is_closed = false;
        }else
            ring.pop_back();

        for(unsigned int j = 0; j < ring.size(); j++)
            neighborhood_list[i].push_back( ring[j] );

        _nb_edges += ring.size();
    }// END FOR( EACH VERTEX )

    // Copy results on a more GPU friendly layout for future use
    delete[] _edge_list;
    delete[] _edge_list_offsets;
    _edge_list = new int[_nb_edges];
    _edge_list_offsets = new int[2*_nb_vert];

    int k = 0;
    for(int i = 0; i < _nb_vert; i++)
    {
        int size = neighborhood_list[i].size();
        _edge_list_offsets[2 * i    ] = k;
        _edge_list_offsets[2 * i + 1] = size;
        for(int j = 0; j <  size; j++)
            _edge_list[k++] = neighborhood_list[i][j];
    }

    if(!_is_closed) std::cout << "Mesh is not a closed mesh\n";

    std::cout << "Mesh edges computed in: " << t.stop() << " sec" << std::endl;
}

// -----------------------------------------------------------------------------

void Mesh::check_integrity()
{
#ifndef NDEBUG
    for(int i = 0; i<_nb_vert; ++i)
    {
        int dep = get_edge_offset(i*2  );
        int off = get_edge_offset(i*2+1);
        // Disconnected vertices do not have edges
        if(is_disconnect(i)) assert( off == 0 );
        //std::cout << off << std::endl;
        for(int j=dep; j< (dep+off); ++j )
        {
            int edge_id = get_edge(j);
            if(edge_id == i){
                assert(false); // no self edges
            }
            // Check edge ids;
            if(edge_id >= 0 && edge_id >= _nb_vert){
                assert(false);
            }

            // Check edge loop integrity
            // (every edge must point to a unique vertex)
            // FIXME: quad pairs are bugged and this condition will fail
/*
            for(int n=dep; n< (dep+off); ++n ) {
                if(n == j) continue;
                int nedge_id = get_edge(n);
                assert(nedge_id != edge_id);
            }
*/
        }
    }
#endif
}

// -----------------------------------------------------------------------------

void Mesh::diffuse_along_mesh(float* vertices_attributes,
                              float locked_value,
                              int nb_iter) const
{
    float* new_attribs   = new float[_nb_vert];
    int*   nb_neighbours = new int  [_nb_vert];
    for(int k = 0; k < nb_iter; k++)
    {
        for(int i = 0; i < _nb_vert; i++){
            new_attribs[i] = 0.f;
            nb_neighbours[i] = 0;
        }
        for(int i = 0; i < _nb_tri; i++){
            int a = _tri[3 * i];
            int b = _tri[3 * i + 1];
            int c = _tri[3 * i + 2];
            new_attribs[a] += vertices_attributes[b] + vertices_attributes[c];
            new_attribs[b] += vertices_attributes[c] + vertices_attributes[a];
            new_attribs[c] += vertices_attributes[a] + vertices_attributes[b];
            nb_neighbours[a] += 2;
            nb_neighbours[b] += 2;
            nb_neighbours[c] += 2;
        }
        for(int i = 0; i < _nb_quad; i++){
            int a = _quad[4 * i];
            int b = _quad[4 * i + 1];
            int c = _quad[4 * i + 2];
            int d = _quad[4 * i + 3];
            new_attribs[a] += vertices_attributes[b] + vertices_attributes[d];
            new_attribs[b] += vertices_attributes[c] + vertices_attributes[a];
            new_attribs[c] += vertices_attributes[b] + vertices_attributes[d];
            new_attribs[d] += vertices_attributes[c] + vertices_attributes[a];
            nb_neighbours[a] += 2;
            nb_neighbours[b] += 2;
            nb_neighbours[c] += 2;
            nb_neighbours[d] += 2;
        }
        for(int i = 0; i < _nb_vert; i++){
            if(vertices_attributes[i] != locked_value){
                vertices_attributes[i] = new_attribs[i] / nb_neighbours[i];
            }
        }
    }
    delete[] new_attribs;
    delete[] nb_neighbours;
}

// -----------------------------------------------------------------------------

void Mesh::diffuse_along_mesh(float* vertices_attributes, int nb_iter) const
{
    float* new_attribs   = new float[_nb_vert];
    int*   nb_neighbours = new int[_nb_vert];
    for(int k = 0; k < nb_iter; k++){
        for(int i = 0; i < _nb_vert; i++){
            new_attribs  [i] = 0.f;
            nb_neighbours[i] = 0;
        }
        for(int i = 0; i < _nb_tri; i++){
            int a = _tri[3 * i    ];
            int b = _tri[3 * i + 1];
            int c = _tri[3 * i + 2];
            new_attribs[a] += vertices_attributes[b] + vertices_attributes[c];
            new_attribs[b] += vertices_attributes[c] + vertices_attributes[a];
            new_attribs[c] += vertices_attributes[a] + vertices_attributes[b];
            nb_neighbours[a] += 2;
            nb_neighbours[b] += 2;
            nb_neighbours[c] += 2;
        }
        for(int i = 0; i < _nb_quad; i++){
            int a = _quad[4 * i    ];
            int b = _quad[4 * i + 1];
            int c = _quad[4 * i + 2];
            int d = _quad[4 * i + 3];
            new_attribs[a] += vertices_attributes[b] + vertices_attributes[d];
            new_attribs[b] += vertices_attributes[c] + vertices_attributes[a];
            new_attribs[c] += vertices_attributes[b] + vertices_attributes[d];
            new_attribs[d] += vertices_attributes[c] + vertices_attributes[a];
            nb_neighbours[a] += 2;
            nb_neighbours[b] += 2;
            nb_neighbours[c] += 2;
            nb_neighbours[d] += 2;
        }
        for(int i = 0; i < _nb_vert; i++)
            vertices_attributes[i] = new_attribs[i] / nb_neighbours[i];
    }
    delete[] new_attribs;
    delete[] nb_neighbours;
}

// -----------------------------------------------------------------------------

void Mesh::add_noise(int fq, float amp){
    for(int i = 0; i < _nb_vert; i++){
        const Vec3_cu c = get_vertex(i);
        float u = c.x;
        float r = sqrtf(c.y * c.y + c.z * c.z);
        float v = atan2(c.y, c.z);
        float dr = cosf(u*fq/r) * cosf(v*fq) * amp;
        _vert[3*i  ] += (c.x * dr);
        _vert[3*i+1] += (c.y * dr);
        _vert[3*i+2] += (c.z * dr);
    }
    // update the vbo
    update_unpacked_vert();
}

// -----------------------------------------------------------------------------

void Mesh::set_point_color_bo(int i, float r, float g, float b, float a){
    assert(i < _nb_vert);
    float* color_ptr;
    _point_color_bo.map_to(color_ptr, GL_WRITE_ONLY);

    const Packed_data d = _packed_vert_map[i];
    for(int j = 0; j < d.nb_ocurrence; j++)
    {
        const int p_idx = d.idx_data_unpacked + j;
        color_ptr[p_idx*4  ] = r;
        color_ptr[p_idx*4+1] = g;
        color_ptr[p_idx*4+2] = b;
        color_ptr[p_idx*4+3] = a;
    }
    _point_color_bo.unmap();
}

// -----------------------------------------------------------------------------

void Mesh::set_color_bo(float r, float g, float b, float a){
    const int size = _size_unpacked_vert_array;
    float* colors = new float[4 * size];
    for(int i = 0; i < size; i++)
    {
        colors[i*4  ]  = r;
        colors[i*4+1]  = g;
        colors[i*4+2]  = b;
        colors[i*4+3]  = a;
    }
    _color_bo.set_data( 4 * size, colors, GL_STATIC_DRAW);
    delete[] colors;
}

// -----------------------------------------------------------------------------

void Mesh::set_point_color_bo(float r, float g, float b, float a)
{
    const int size = _size_unpacked_vert_array;
    float* colors = new float[4 * size];
    for(int i = 0; i < size; i++){
        colors[i*4  ]  = r;
        colors[i*4+1]  = g;
        colors[i*4+2]  = b;
        colors[i*4+3]  = a;
    }
    _point_color_bo.set_data( 4 * size, colors, GL_STATIC_DRAW);
    delete[] colors;
}

// -----------------------------------------------------------------------------

void Mesh::init_vbos()
{
    assert(_vert != 0);
    assert(_size_unpacked_vert_array != 0);
    assert(_nb_vert > 0);

    _vbo.set_data(_size_unpacked_vert_array*3, 0, GL_STATIC_DRAW);
    update_unpacked_vert();
    if(_nb_tri > 0)
        _index_bo_tri. set_data( 3 * _nb_tri , _unpacked_tri , GL_STATIC_DRAW);

    if(_nb_quad > 0)
        _index_bo_quad.set_data( 4 * _nb_quad, _unpacked_quad, GL_STATIC_DRAW);

    const int size = _size_unpacked_vert_array;
    if(_has_normals){
        _normals_bo. set_data(3 * size, _normals , GL_STATIC_DRAW);
        _tangents_bo.set_data(3 * size, _tangents, GL_STATIC_DRAW);
    }

    if(_has_tex_coords)
        _tex_bo.set_data(2 * size, _tex_coords, GL_STATIC_DRAW);

    float* colors      = new float[ 4 * size];
    int*   point_index = new int  [ _nb_vert ];
    for(int i = 0; i < size; i++)
    {
         colors[i*4] = colors[i*4+1] = colors[i*4+2] = colors[i*4+3] = 1.f;
        if( i < _nb_vert)
            point_index[i] = _packed_vert_map[i].idx_data_unpacked;
    }
    _index_bo_point.set_data(     _nb_vert, point_index, GL_STATIC_DRAW);
    _point_color_bo.set_data( 4 * size   , colors     , GL_STATIC_DRAW);

    for(int i = 0; i < size; i++) colors[i*4+3] = 0.99f;

    _color_bo.set_data( 4 * size, colors, GL_STATIC_DRAW);

    delete[] colors;
    delete[] point_index;

    // Register VBOs
    _vbo.cuda_register();
    _normals_bo.cuda_register();
    _color_bo.cuda_register();

    if(_nb_tri  > 0) _index_bo_tri. cuda_register();
    if(_nb_quad > 0) _index_bo_quad.cuda_register();

    if(_has_tex_coords)
    {
        _tex_bo.cuda_register();
        _tangents_bo.cuda_register();
    }
}

// -----------------------------------------------------------------------------

void Mesh::update_unpacked_vert()
{
    float* vert_gpu = 0;
    _vbo.map_to(vert_gpu, GL_WRITE_ONLY);
    for(int i = 0; i < _nb_vert; i++)
    {
        const Packed_data d = _packed_vert_map[i];
        for(int j = 0; j < d.nb_ocurrence; j++)
        {
            const int p_idx = d.idx_data_unpacked + j;
            vert_gpu[p_idx*3    ] = _vert[i*3    ];
            vert_gpu[p_idx*3 + 1] = _vert[i*3 + 1];
            vert_gpu[p_idx*3 + 2] = _vert[i*3 + 2];
        }
    }
    _vbo.unmap();
}

// -----------------------------------------------------------------------------

Vec3_cu Mesh::get_normal(int i, int n) const {
    assert(_has_normals);
    Packed_data d = _packed_vert_map[i];
    int idx = d.idx_data_unpacked + n;
    if(d.nb_ocurrence == 0)
        return Vec3_cu(0.f, 0.f, 0.f);
    assert(n < d.nb_ocurrence);
    return Vec3_cu(_normals[3*idx], _normals[3*idx+1], _normals[3*idx+2]);
}

// -----------------------------------------------------------------------------

Vec3_cu Mesh::get_mean_normal(int i) const {
    assert(_has_normals);
    Packed_data d = _packed_vert_map[i];
    Vec3_cu mean(0.f, 0.f, 0.f);
    for (int n = 0; n < d.nb_ocurrence; ++n) {
        int idx = d.idx_data_unpacked + n;
        Vec3_cu nor(_normals[3*idx], _normals[3*idx+1], _normals[3*idx+2]);
        mean += nor;
    }
    return mean;
}

// -----------------------------------------------------------------------------

Mesh::Tex_coords Mesh::get_tex_coords(int i, int n) const {
    assert(_has_tex_coords);
    int idx = _packed_vert_map[i].idx_data_unpacked + n;
    assert(n < _packed_vert_map[i].nb_ocurrence);
    return Tex_coords(_tex_coords[2*idx], _tex_coords[2*idx+1]);
}

// -----------------------------------------------------------------------------

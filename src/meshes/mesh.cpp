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

#include "macros.hpp"
#include "mesh.hpp"
#include "loader_mesh.hpp"
#include "timer.hpp"
#include "std_utils.hpp"

Mesh::Mesh(const Mesh& m) :
    _is_initialized(m._is_initialized),
    _has_normals(m._has_normals),
    _offset(m._offset),
    _scale(m._scale),
    _max_faces_per_vertex(m._max_faces_per_vertex),
    _nb_vert(m._nb_vert),
    _nb_tri(m._nb_tri),
    _nb_edges(m._nb_edges),
    _tri_list_per_vert(m._tri_list_per_vert),
    _size_unpacked_vert_array(m._size_unpacked_vert_array)
{

    _vert            = new float[ _nb_vert*3 ];
    _is_connected    = new bool [ _nb_vert   ];
    _is_side         = new bool [ _nb_vert*3 ];
    _packed_vert_map = new Packed_data[_nb_vert];

    _tri       = new int[3 * _nb_tri];
    _piv       = new int[4 * _nb_tri];
    _edge_list = new int[_nb_edges];

    _edge_list_offsets = new int[2*_nb_vert];

    _normals    = _has_normals     ? new float [_size_unpacked_vert_array * 3] : 0;

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
        _tri[i] = m._tri[i];

    for(int i = 0; i < 4*_nb_tri; i++)
        _piv[i] = m._piv[i];

    for(int i = 0; i < _nb_edges; i++)
        _edge_list[i] = m._edge_list[i];

    if(m._has_normals)
    {
        for(int i = 0; i < _size_unpacked_vert_array; i++)
        {
            if(_has_normals){
                _normals[i*3  ] = m._normals[i*3  ];
                _normals[i*3+1] = m._normals[i*3+1];
                _normals[i*3+2] = m._normals[i*3+2];
            }
        }
    }
}

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
    delete[] _normals;
    delete[] _packed_vert_map;
    delete[] _piv;
    delete[] _edge_list;
    delete[] _edge_list_offsets;
    _is_connected      = 0;
    _vert              = 0;
    _tri               = 0;
    _normals           = 0;
    _packed_vert_map   = 0;
    _piv               = 0;
    _edge_list         = 0;
    _edge_list_offsets = 0;

    _tri_list_per_vert.clear();
}

void Mesh::compute_piv()
{
    delete[] _piv;
    _piv = new int [4 * _nb_tri];
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
    _max_faces_per_vertex = imax;
    delete [] pic;
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
    delete[] new_normals;
}

// -----------------------------------------------------------------------------

void Mesh::compute_face_index()
{
    assert(_size_unpacked_vert_array > 0);

    _tri_list_per_vert.clear();
    _tri_list_per_vert. resize(_nb_vert);

    for(int i = 0; i < _nb_tri; i++){
        for(int j = 0; j < 3; j++){
            int v = _tri[3*i +j ];
            assert(v>=0);
            _tri_list_per_vert[v].push_back(i);
        }
    }
}

// -----------------------------------------------------------------------------

Mesh::Mesh(const Loader::Abs_mesh& mesh):
    _is_initialized(false),
    _has_normals(false),
    _offset(0.f,0.f,0.f),
    _scale(1.f),
    _nb_vert(0),
    _nb_tri(0),
    _nb_edges(0),
    _vert(0),
    _is_connected(0),
    _is_side(0),
    _tri(0),
    _piv(0),
    _edge_list(0),
    _edge_list_offsets(0),
    _normals(0),
    _size_unpacked_vert_array(-1),
    _packed_vert_map(0)
{
    _is_initialized = false;
    free_mesh_data();

    _nb_vert = (int) mesh._vertices.size();
    _nb_tri  = (int) mesh._triangles.size();

    _vert          = new float [_nb_vert * 3];
    _tri           = new int   [_nb_tri  * 3];

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
    std::vector<std::map<int,int> > pair_per_vert(_nb_vert);
    std::vector<int> nb_pair_per_vert(_nb_vert, 0);
    for( int i = 0; i < _nb_tri; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            // Fill triangle index
            int v_idx = mesh._triangles[i].v[j];
            int n_idx = mesh._triangles[i].n[j];

            _is_connected[v_idx] = true;
            _tri[i*3+j] = v_idx;

            std::map<int,int>& map = pair_per_vert[v_idx];
            if( map.find(n_idx) == map.end() )
            {
                map[n_idx] = nb_pair_per_vert[v_idx];
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

    // Copy triangles index, normals and texture coordinates
    _has_normals    = false;
    for( int i = 0; i < _nb_tri; i++)
    {
        for( int j = 0; j < 3; j++)
        {
            int v_idx = mesh._triangles[i].v[j];
            int n_idx = mesh._triangles[i].n[j];

            int off = pair_per_vert[v_idx][n_idx];

            assert(off < _packed_vert_map[v_idx].nb_ocurrence);

            int v_unpacked = _packed_vert_map[v_idx].idx_data_unpacked + off;

            // Fill normal as there index match the unpacked vertex array
            if( n_idx != -1 )
                *((Loader::Normal*)(_normals+v_unpacked*3)) = mesh._normals[n_idx];
            else
                *((Loader::Normal*)(_normals+v_unpacked*3)) = Loader::Normal();

            _has_normals    = _has_normals    || (n_idx != -1);
        }
    }

    // XXX: We could ignore input normals, so we wouldn't be affected by normals set to special
    // values for lighting purposes and we wouldn't interact as much with the host.
    if( !_has_normals )
        compute_normals();
    // Initialize VBOs
    compute_piv();
    compute_face_index();
    compute_edges();
    _is_initialized = true;
}

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
        // We suppose the faces are tris to reserve memory
        if( _max_faces_per_vertex > 0)
            neighborhood_list.reserve(_max_faces_per_vertex*3);

        if( is_disconnect(i) ) continue; // TODO should be is_side = true no ?

        list_pairs.clear();
        // fill pairs with the first ring of neighborhood of quads and triangles
        for(unsigned int j=0; j<_tri_list_per_vert[i].size(); j++)
            list_pairs.push_back(pair_from_tri(_tri_list_per_vert[i][j], i));

        // Try to build the ordered list of the first ring of neighborhood of i
        std::deque<int> ring;
        ring.push_back(list_pairs[0].first );
        ring.push_back(list_pairs[0].second);
        std::vector<std::pair<int, int> >::iterator it = list_pairs.begin();
        list_pairs.erase(it);
        size_t pairs_left = list_pairs.size();
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
        }

        if(ring[0] != ring[ring.size()-1]){
            _is_side[i] = true;
        }else
            ring.pop_back();

        for(unsigned int j = 0; j < ring.size(); j++)
            neighborhood_list[i].push_back( ring[j] );

        _nb_edges += (int) ring.size();
    }// END FOR( EACH VERTEX )

    // Copy results on a more GPU friendly layout for future use
    delete[] _edge_list;
    delete[] _edge_list_offsets;
    _edge_list = new int[_nb_edges];
    _edge_list_offsets = new int[2*_nb_vert];

    int k = 0;
    for(int i = 0; i < _nb_vert; i++)
    {
        int size = (int) neighborhood_list[i].size();
        _edge_list_offsets[2 * i    ] = k;
        _edge_list_offsets[2 * i + 1] = size;
        for(int j = 0; j <  size; j++)
            _edge_list[k++] = neighborhood_list[i][j];
    }

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

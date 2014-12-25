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
#include "graph.hpp"

// -----------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <stack>

#include "std_utils.hpp"

static bool rec_is_cycles(std::vector<bool>& seen,
                          const std::vector< std::vector< int > > neighs,
                          int curr,
                          int pt)
{
    // Already seen it's a cycle
    if( seen[curr] ) return true;

    seen[curr] = true;

    // leaf
    if( neighs[curr].size() < 2 && pt != -1) return false;

    bool cycle = false;
    for (unsigned i = 0; i < neighs[curr].size(); ++i) {

        if(neighs[curr][i] != pt)
            cycle = cycle || rec_is_cycles(seen, neighs, neighs[curr][i], curr);
    }
    return cycle;
}

// -----------------------------------------------------------------------------

bool Graph::is_cycles(int root) const
{
    if( _neighs[root].size() < 1) return false;

    std::vector<bool> seen(_vertices.size(), false);
    return  rec_is_cycles(seen, _neighs, root, -1);
}

// -----------------------------------------------------------------------------

int Graph::push_vertex(const Vec3_cu& v){
    _vertices.push_back( v );
    _neighs.  push_back( std::vector<int>() );
    return _vertices.size() - 1;
}

// -----------------------------------------------------------------------------

int Graph::push_edge(const Edge& e){
    assert(e.a >= 0 && e.a < (int)_vertices.size());
    assert(e.b >= 0 && e.b < (int)_vertices.size());

    if( Std_utils::exists(_edges, e) )
    {
        std::cerr << "WARNING: edge already exists in the graph" << std::endl;
        return -1;
    }

    _edges.push_back( e );
    _neighs[e.a].push_back(e.b);
    _neighs[e.b].push_back(e.a);

    return _edges.size() - 1;
}

// -----------------------------------------------------------------------------

void Graph::remove_vertex(int to_remove)
{
    if(to_remove > -1 && to_remove < (int)_vertices.size())
    {
        _neighs.clear();
        Std_utils::pop(_vertices, to_remove);
        _neighs.resize( _vertices.size() );
        // Remove edges connected to i and replace index k to i
        int k = _vertices.size();
        for(unsigned j = 0; j < _edges.size(); j++)
        {
            while((j < _edges.size()) &&
                  (_edges[j].a == to_remove || _edges[j].b == to_remove)  )
            {
                Std_utils::pop(_edges, j);
            }

            if( j < _edges.size() )
            {
                if(_edges[j].a == k )
                    _edges[j].a = to_remove;

                if(_edges[j].b == k )
                    _edges[j].b = to_remove;

                // Rebuild _neighs :
                Edge e = _edges[j];
                _neighs[e.a].push_back(e.b);
                _neighs[e.b].push_back(e.a);
            }
        }
    }
}

// -----------------------------------------------------------------------------

void Graph::clear()
{
    _vertices.clear();
    _edges.   clear();
    _neighs.  clear();
    _offset = Vec3_cu::zero();
    _scale  = 1.f;
}

void Graph::set_offset_scale(const Vec3_cu& offset, float scale)
{
    // TODO: don't change vertices list and apply offset and scale through
    // accessors
    this->_offset = offset;
    this->_scale  = scale;

    for(unsigned i = 0; i < _vertices.size(); i++)
    {
        Vec3_cu& v = get_vertex(i);

        float x = (v.x + offset.x) * scale;
        float y = (v.y + offset.y) * scale;
        float z = (v.z + offset.z) * scale;

        v.x = x; v.y = y; v.z = z;
    }
}

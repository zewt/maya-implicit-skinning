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

// -----------------------------------------------------------------------------

#include "std_utils.hpp"
#include "glassert.h"
#include "camera.hpp"
#include "port_glew.h"

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

int Graph::get_window_nearest(float x, float y, float& dist)
{
    int res = -1;
    float dst2 = std::numeric_limits<float>::infinity();
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX , modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    GLdouble vx, vy, vz;
    for(unsigned i = 0; i < _vertices.size(); i++)
    {
        Vec3_cu& v = _vertices[i];
        GLdouble ivx = (GLdouble)v.x, ivy = (GLdouble)v.y, ivz = (GLdouble)v.z;

        gluProject(ivx, ivy, ivz, modelview, projection, viewport, &vx, &vy, &vz);

        float dx = (float)vx - x;
        float dy = (float)vy - y;
        float d2 = dx * dx + dy * dy;
        if(d2 < dst2){
            dst2 = d2;
            res = i;
        }
    }
    dist = sqrtf(dst2);
    return res;
}

// -----------------------------------------------------------------------------

void Graph::draw(const Camera& cam, int current_vertex) const
{
    glAssert( glMatrixMode(GL_MODELVIEW) );
    Vec3_cu vx   = cam.get_x();
    Vec3_cu vy   = cam.get_y();
    Vec3_cu cdir = cam.get_dir();
    Vec3_cu pcam = cam.get_pos();
    for(unsigned i = 0; i < _vertices.size(); i++)
    {
        glAssert( glPushMatrix() );
        const Vec3_cu& v = _vertices[i];
        float dst = (pcam - v).norm();
        float fc = 0.008f * dst;
        glAssert( glTranslatef(v.x, v.y, v.z) );
        if((int)i == current_vertex){
            glAssert( glRotatef(45.f, cdir.x, cdir.y, cdir.z) );
            glAssert( glColor4f(1.f, 1.f, 0.2f, 1.f) );
        }else
            glAssert( glColor4f(1.f, 0.f, 0.f, 1.f) );

        glBegin(GL_QUADS);{
            glVertex3f(-vx.x * fc, -vx.y * fc, -vx.z * fc);
            glVertex3f(-vy.x * fc, -vy.y * fc, -vy.z * fc);
            glVertex3f( vx.x * fc,  vx.y * fc,  vx.z * fc);
            glVertex3f( vy.x * fc,  vy.y * fc,  vy.z * fc);
        }glAssert( glEnd() );
        glAssert( glPopMatrix() );
    }
    glAssert( glColor4f(1.f, 0.f, 0.f, 1.f) );
    glAssert( glLineWidth(3.f) );
    if(_vertices.size() > 0 && _edges.size() > 0)
    {
        glAssert( glEnableClientState(GL_VERTEX_ARRAY) );
        glAssert( glVertexPointer(3, GL_FLOAT, 0, &(_vertices[0])) );
        glAssert( glDrawElements(GL_LINES, _edges.size() *2, GL_UNSIGNED_INT, &(_edges[0])) );
        // Release pointer
        glAssert( glVertexPointer(3, GL_FLOAT, 0, 0) );
        glAssert( glDisableClientState(GL_VERTEX_ARRAY) );
    }
}

// -----------------------------------------------------------------------------

void Graph::save_to_file(const char* filename) const
{
    using namespace std;
    ofstream file;
    file.open(filename);
    file << _vertices.size() << ' ' << _edges.size() << endl;
    for(unsigned i = 0; i < _vertices.size(); i++){
        Vec3_cu v = _vertices[i];

        // DEBUG --------------------------------------------
        // We should save as is original vertices but unfortunatly
        // scale and offset are directly applied to vertices list ...
        //
        float x = (v.x * (1.f / _scale) - _offset.x);
        float y = (v.y * (1.f / _scale) - _offset.y);
        float z = (v.z * (1.f / _scale) - _offset.z);
        v.x = x; v.y = y; v.z = z;
        // DEBUG -------------------------------------------------

        file << v.x << ' ' << v.y << ' ' << v.z << endl;
    }
    for(unsigned i = 0; i < _edges.size(); i++){
        const Edge& e = _edges[i];
        file << e.a << ' ' << e.b << '\n';
    }
    cout << "done" << endl;
}

// -----------------------------------------------------------------------------

void Graph::load_from_file(const char* filename)
{
    using namespace std;
    ifstream file(filename);
    if( !file.is_open() ){
        cerr << "Error opening file: " << filename << endl;
        return;
    }
    _vertices.clear();
    _edges.clear();
    _neighs.clear();

    int nb_vert, nb_edges;
    file >> nb_vert;
    file >> nb_edges;
    cout << "nb vertices: " << nb_vert << endl << "nb edges: " << nb_edges << endl;
    for(int i = 0; i < nb_vert; i++){
        float vx, vy, vz;
        file >> vx; file >> vy; file >> vz;
        push_vertex(Vec3_cu(vx,vy,vz));
    }

    for(int i = 0; i < nb_edges; i++){
        int a, b;
        file >> a; file >> b;
        push_edge(Edge(a,b));
    }

    cout << "done" << endl;

    file.close();
}

// -----------------------------------------------------------------------------

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

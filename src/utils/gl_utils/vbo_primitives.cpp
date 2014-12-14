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
#include "vbo_primitives.hpp"

#include <cassert>
#include "glsave.hpp"

// -----------------------------------------------------------------------------

VBO_primitives::~VBO_primitives()
{
    std::deque< GlBuffer_obj* >::iterator it;
    for( it = _vbos.begin() ; it != _vbos.end(); ++it ) delete (*it);
    for( it = _nbos.begin() ; it != _nbos.end(); ++it ) delete (*it);
    for( it = _ibos.begin() ; it != _ibos.end(); ++it ) delete (*it);
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_sphere(float radius, int res){
    Gen_mesh::Tri_Mesh_data* sphere = Gen_mesh::sphere(radius, res);
    Prim_id id = add_primitive(*sphere);
    delete sphere;
    return id;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_circle(float radius, int res, float diameter){
    Gen_mesh::Line_data* circle = Gen_mesh::circle(radius, res, diameter);
    Prim_id id = add_primitive(*circle);
    delete circle;
    return id;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_arc_circle(float radius, int res, float diameter){
    Gen_mesh::Line_data* circle = Gen_mesh::circle(radius, res, diameter);
    // Remove last segment :
    circle->nb_line -= 1;
    Prim_id id = add_primitive(*circle);
    delete circle;
    return id;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_grid(float width, float height, int res_x, int res_y){
    Gen_mesh::Line_data* grid = Gen_mesh::grid(width, height, res_x, res_y);
    Prim_id id = add_primitive(*grid);
    delete grid;
    return id;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_cylinder(float radius, float length, int slices, int res){
    Gen_mesh::Tri_Mesh_data* cylinder = Gen_mesh::cylinder(length, radius, slices, res, false);
    Prim_id id = add_primitive(*cylinder);
    delete cylinder;
    return id;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_cylinder_cage(float radius, float length, int caps_res, int body_res){
    Gen_mesh::Line_data* cyl_cage = Gen_mesh::cylinder_cage(length, radius, caps_res, body_res);
    Prim_id id = add_primitive(*cyl_cage);
    delete cyl_cage;
    return id;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::init_cube(){
    Gen_mesh::Quad_Mesh_data* cube = Gen_mesh::cube();
    Prim_id id = add_primitive(*cube);
    delete cube;
    return id;
}

// -----------------------------------------------------------------------------

void VBO_primitives::draw(Prim_id id) const
{
    glAssert( glEnableClientState(GL_VERTEX_ARRAY) );
    _vbos[id]->bind();
    glAssert( glVertexPointer(3, GL_FLOAT, 0, 0) );

    _nbos[id]->bind();
    glAssert( glEnableClientState(GL_NORMAL_ARRAY) );
    glAssert( glNormalPointer(GL_FLOAT, 0, 0) );

    _ibos[id]->bind();
    switch(_type[id])
    {
    case QUAD :
    glAssert(glDrawElements(GL_QUADS, 4*_nb_elt[id], GL_UNSIGNED_INT, 0));
    break;
    case TRIANGLE :
    glAssert(glDrawElements(GL_TRIANGLES, 3*_nb_elt[id], GL_UNSIGNED_INT, 0));
    break;
    case LINE :
    glAssert(glDrawElements(GL_LINES, 2*_nb_elt[id], GL_UNSIGNED_INT, 0));
    break;
    case LINE_STRIP :
    glAssert(glDrawElements(GL_LINE_STRIP, _nb_elt[id], GL_UNSIGNED_INT, 0));
    break;
    }
    _ibos[id]->unbind();

    glAssert( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    glAssert( glVertexPointer(3, GL_FLOAT, 0, 0) );
    glAssert( glNormalPointer(GL_FLOAT, 0, 0) );
    glAssert( glDisableClientState(GL_VERTEX_ARRAY) );
    glAssert( glDisableClientState(GL_NORMAL_ARRAY) );

}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::add_primitive(const Gen_mesh::Tri_Mesh_data& mesh)
{
    _vbos.push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)         );
    _nbos.push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)         );
    _ibos.push_back( new GlBuffer_obj(GL_ELEMENT_ARRAY_BUFFER) );
    _nb_elt.push_back( mesh.nb_tri );
    _type.push_back(TRIANGLE);

    _vbos.back()->set_data(mesh.nb_vert*3, mesh.vertex , GL_STATIC_DRAW);
    _nbos.back()->set_data(mesh.nb_vert*3, mesh.normals, GL_STATIC_DRAW);
    _ibos.back()->set_data(mesh.nb_tri*3 , mesh.index  , GL_STATIC_DRAW);

    return _vbos.size()-1;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::add_primitive(const Gen_mesh::Quad_Mesh_data& mesh)
{
    _vbos.push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)         );
    _nbos.push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)         );
    _ibos.push_back( new GlBuffer_obj(GL_ELEMENT_ARRAY_BUFFER) );
    _nb_elt.push_back( mesh.nb_quad );
    _type.push_back(QUAD);

    _vbos.back()->set_data(mesh.nb_vert*3, mesh.vertex , GL_STATIC_DRAW);
    _nbos.back()->set_data(mesh.nb_vert*3, mesh.normals, GL_STATIC_DRAW);
    _ibos.back()->set_data(mesh.nb_quad*4, mesh.index  , GL_STATIC_DRAW);

    return _vbos.size()-1;
}

// -----------------------------------------------------------------------------

Prim_id VBO_primitives::add_primitive(const Gen_mesh::Line_data& mesh,
                                      bool strip)
{
    _vbos.push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)         );
    _nbos.push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)         );
    _ibos.push_back( new GlBuffer_obj(GL_ELEMENT_ARRAY_BUFFER) );
    _nb_elt.push_back( mesh.nb_line + (strip ? 1 : 0) );
    _type.push_back(strip ? LINE_STRIP : LINE);

    _vbos.back()->set_data(mesh.nb_vert*3, mesh.vertex , GL_STATIC_DRAW);
    _nbos.back()->set_data(mesh.nb_vert*3, mesh.normals, GL_STATIC_DRAW);
    _ibos.back()->set_data(strip ? mesh.nb_line+1 : mesh.nb_line*2, mesh.index, GL_STATIC_DRAW);

    return _vbos.size()-1;
}

// -----------------------------------------------------------------------------

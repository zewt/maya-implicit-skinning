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
#ifndef VBO_PRIMITIVES_HPP__
#define VBO_PRIMITIVES_HPP__

#include <deque>
#include "glbuffer_object.hpp"
#include "gen_mesh.hpp"


/** @brief Handle primitive initialization and drawing on GPU
  This class provide various primitive initialization and drawing with opengl
  buffer objects. Some are predifined such as cone cylinder and sphere but
  you can create on GPU the desire primitive with custom tesselation and size
*/

typedef int Prim_id;

class VBO_primitives {
public:
    VBO_primitives(){ }
    ~VBO_primitives();

    enum Prim_type {
        TRIANGLE, LINE, LINE_STRIP, QUAD
    };

    //==========================================================================
    /// @name Initialization of custom primitives
    //==========================================================================
    /// @{
    Prim_id init_sphere(float radius, int res);
    Prim_id init_circle(float radius, int res, float diameter = 2.f * M_PI);
    Prim_id init_arc_circle(float radius, int res, float diameter = 2.f * M_PI);
    Prim_id init_grid(float width, float height, int res_x, int res_y);
    Prim_id init_cylinder(float radius, float length, int slices, int res);
    Prim_id init_cylinder_cage(float radius, float length, int caps_res, int body_res);
    Prim_id init_cube();
    /// @}

    Prim_id add_primitive(const Gen_mesh::Tri_Mesh_data& mesh);
    Prim_id add_primitive(const Gen_mesh::Quad_Mesh_data& mesh);
    /// @param strip : wether the mesh composed of a single line strip or lines
    Prim_id add_primitive(const Gen_mesh::Line_data& mesh, bool strip = false);

    //==========================================================================
    /// @name Drawing
    //==========================================================================

    /// Draw a custom primitive
    /// @param id  primitive id returned from a init_xx() method
    void draw(Prim_id id) const;

private:
    //==========================================================================
    /// @name List of opengl buffer objects for custom primitives
    //==========================================================================
    /// @{
    std::deque< GlBuffer_obj* > _vbos; ///< Vertex buffer object
    std::deque< GlBuffer_obj* > _nbos; ///< Normal buffer object
    std::deque< GlBuffer_obj* > _ibos; ///< Index buffer object
    std::deque<int> _nb_elt;
    std::deque<Prim_type> _type;
    /// @}
};

#endif // VBO_PRIMITIVES_HPP__

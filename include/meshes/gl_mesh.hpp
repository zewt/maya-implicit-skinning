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
#ifndef GL_MESH_HPP__
#define GL_MESH_HPP__

#include <vector>
#include "port_glew.h"

struct GlBuffer_obj;

/** @brief Utility to display quad mesh with Opengl

*/
class Gl_mesh_quad {
public:

    Gl_mesh_quad();

    ~Gl_mesh_quad();

    /// Build the gl mesh from a vector of points and quad index
    /// @param points   List of points with contigus coordinates (x,y,z)
    /// @param quad_idx List of quad faces
    Gl_mesh_quad(const std::vector<float>& points,
                 const std::vector<int>& quad_idx);

    void set_points(const float* points, int size);

    void set_index(const int* quad_idx, int size);

    /// initialize GPU memory (VBO, VAO ...)
    void compileGl();

    void draw() const;


private:
    void init_bo();

    std::vector<float> _points;   ///< list of points with contigus (x,y,z)
    std::vector<int>   _quad_idx; ///< list of quad faces

    GlBuffer_obj* _vertex_bo;
    // Possible extenssion to be done:
    //BufferObject<GL_ARRAY_BUFFER> _normals_bo;
    //BufferObject<GL_ARRAY_BUFFER> _color_bo;
    //BufferObject<GL_ARRAY_BUFFER> _tex_bo;

    GlBuffer_obj* _index_bo_quad;
};

#endif // GL_MESH_HPP__

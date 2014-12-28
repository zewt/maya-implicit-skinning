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
#ifndef LOADER_MESH_HPP__
#define LOADER_MESH_HPP__

#include <vector>
#include <string>

// =============================================================================
namespace Loader {
// =============================================================================

struct Vertex {
    /// Vertex components
    float x, y, z;

    Vertex() : x(0),y(0),z(0) {}
    Vertex(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }
};

//------------------------------------------------------------------------------

struct Normal {
    /// Normal components
    float x, y, z;

    Normal() : x(0), y(0), z(0) { }
    Normal(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }
};

//------------------------------------------------------------------------------
struct Tri_face {
    unsigned v[3]; ///< vertex indices for the triangle
    int n[3];      ///< normal indices for the triangle

    /// ctor
    Tri_face() {
        v[0] = v[1] = v[2] =  0;
        // -1 indicates not used
        n[0] = n[1] = n[2] = -1;
    }
};


/**
  @struct Abs_mesh
  @brief An abstract mest only represent triangle meshes.
  An abstract mesh contains the triangulated representation of a mesh
  in '_triangles' but also it's version with quads to enable nicer rendering
  in wireframe mode (attribute _render_faces)
*/
struct Abs_mesh {
    std::vector<Vertex>    _vertices;
    std::vector<Normal>    _normals;
    std::vector<Tri_face>  _triangles;  ///< the triangulated faces

    void clear();
};

} // END LOADER NAMESPACE ======================================================

#endif // LOADER_MESH_HPP__

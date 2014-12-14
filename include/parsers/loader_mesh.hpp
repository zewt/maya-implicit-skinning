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

struct TexCoord {
    float u, v;

    /// ctors
    TexCoord() : u(0),v(0) { }
    TexCoord(float _u, float _v) : u(_u),v(_v) {    }
};

//------------------------------------------------------------------------------

struct Tri_face {
    unsigned v[3]; ///< vertex indices for the triangle
    int n[3];      ///< normal indices for the triangle
    int t[3];      ///< texture coordinate indices for the triangle

    /// ctor
    Tri_face() {
        v[0] = v[1] = v[2] =  0;
        // -1 indicates not used
        n[0] = n[1] = n[2] = -1;
        t[0] = t[1] = t[2] = -1;
    }
};

//------------------------------------------------------------------------------

struct Quad_face {
    unsigned v[4]; ///< vertex indices for the triangle
    int n[4];      ///< normal indices for the triangle
    int t[4];      ///< texture coordinate indices for the triangle

    /// ctor
    Quad_face() {
        v[0] = v[1] = v[2] = v[3] =  0;
        // -1 indicates not used
        n[0] = n[1] = n[2] = n[3] = -1;
        t[0] = t[1] = t[2] = t[3] = -1;
    }

    void triangulate(Tri_face& f0, Tri_face& f1) const
    {
        f0.v[0] = v[0]; f0.v[1] = v[1]; f0.v[2] = v[2];
        f0.n[0] = n[0]; f0.n[1] = n[1]; f0.n[2] = n[2];
        f0.t[0] = t[0]; f0.t[1] = t[1]; f0.t[2] = t[2];

        f1.v[0] = v[0]; f1.v[1] = v[2]; f1.v[2] = v[3];
        f1.n[0] = n[0]; f1.n[1] = n[2]; f1.n[2] = n[3];
        f1.t[0] = t[0]; f1.t[1] = t[2]; f1.t[2] = t[3];
    }
};

//------------------------------------------------------------------------------

struct Material {
    /// material name
    std::string _name;
    /// don't know :| Seems to always be 4
    int _illum;
    float _Ka[4]; ///< ambient
    float _Kd[4]; ///< diffuse
    float _Ks[4]; ///< specular
    float _Tf[3]; ///< transparency
    float _Ni;    ///< intensity
    float _Ns;    ///< specular power
    /// @name Texture path relative to the mesh file
    /// @{
    std::string _map_Ka;   ///< ambient texture map
    std::string _map_Kd;   ///< diffuse texture map
    std::string _map_Ks;   ///< specular texture map
    std::string _map_Bump; ///< bump texture map
    /// @}
    /// bump map depth. Only used if bump is relevent.
    float _Bm;

    /// ctors
    Material();
    Material(const Material& mat);
    /// dtor
    ~Material();

    /// set all maps path relative to 'p'
    void set_relative_paths(const std::string& p);
};

//------------------------------------------------------------------------------

struct MaterialGroup {
    /// the material applied to a set of faces
    unsigned _material_idx;
    /// the starting index of the face to which the material is applied
    unsigned _start_face;
    /// the ending index of the face to which the material is applied
    unsigned _end_face;

    /*
    /// start index for points to which the material is applied
    unsigned _start_point;
    /// end index for points to which the material is applied
    unsigned _end_point;
    */

    /// ctors
    MaterialGroup() : _material_idx(0), _start_face(0), _end_face(0) { }

    MaterialGroup(int idx, int start, int end) :
        _material_idx(idx), _start_face(start), _end_face(end)
    { }
};

//------------------------------------------------------------------------------

struct Group {
    /// start index for faces in the group
    unsigned _start_face;
    /// end index for faces in the group
    unsigned _end_face;

    /*
    /// start index for points in the group (surface)
    unsigned start_point;
    /// end index for points in the group (surface)
    unsigned _end_point;
    */

    /// name of the group
    std::string _name;

    /// a set of material groupings within this surface. ie, which
    /// materials are assigned to which faces within this group.
    std::vector<MaterialGroup> _assigned_mats;

    /// ctors
    Group() : _start_face(0),_end_face(0),_name(""),_assigned_mats() {    }

    Group(const std::string& name, int start, int end) :
        _start_face(start), _end_face(end), _name(name), _assigned_mats() {    }
};

//------------------------------------------------------------------------------

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
    std::vector<TexCoord>  _texCoords;
    std::vector<Tri_face>  _triangles;  ///< the triangulated faces
    std::vector<Group>     _groups;
    std::vector<Material>  _materials;  ///< faces materials for '_triangles'

    struct Render_faces {
        std::vector<Tri_face > _tris;
        std::vector<Quad_face> _quads;
    };

    /// Mesh with quad and triangle faces.
    /// (usefull to render quads in wireframe mode)
    /// @note for now no material are attached to it.
    /// Any other face type must be triangulated
    Render_faces _render_faces;

    void clear();
};

} // END LOADER NAMESPACE ======================================================

#endif // LOADER_MESH_HPP__

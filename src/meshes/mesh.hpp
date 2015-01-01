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
#ifndef MESH_HPP__
#define MESH_HPP__

#include <cstdio>
#include <cmath>
#include <vector>
#include <cassert>
#include <algorithm>

#include "vec3_cu.hpp"

namespace Loader{
    struct Abs_mesh;
}

// END FORWARD DEFINITIONS  ----------------------------------------------------



/** @brief Mesh utility drawing, loading, smoothing etc.

  This class has been primarly designed to handle static mesh. It would be
  foolish try implementing topological changes (adding/suppressing vertices
  triangles edges etc.)


  How to look up the first ring neighborhood of every vertex :
  @code

  Mesh m;
  ...
  for(int i = 0; i < m.get_nb_vertices(); i++)
  {
      int dep      = m.get_edge_offset(i*2    );
      int nb_neigh = m.get_edge_offset(i*2 + 1);
      for(int n = dep; n < (dep+nb_neigh); n++)
      {
          int index_neigh = m.get_edge(n);
          ...
          Vec3_cu v = m.get_vertex(index_neigh); // 3d position
          ...
      }
  }
  @endcode

  Sometime a mesh vertex has multiple textures coordinates at the seams of two
  texture or multiples normals at sharp edges. To handle these we use packed and
  unpacked data:

  Packed data are the mesh vertices and face index unchanged from the file we
  imported them. The unpacked data is the copy of this mesh but with duplicate
  vertices each time there is several normal or uv coordinates. This allows
  to display easily the mesh with the unpacked data.

  A mapping between packed and unpacked data is stored in 'packed_vert_map' so
  that one can update the unpacked data if vertices position in the packed
  arrays ever changes.
 */
class Mesh{
public:

    //  ------------------------------------------------------------------------
    /// @name Inner structs
    //  ------------------------------------------------------------------------

    struct Packed_data{
        /// index of the data in the unpacked array
        int idx_data_unpacked;
        /// number of occurences consecutive to idx_data_unpacked in the
        /// unpacked array
        int nb_ocurrence;
    };

    /// @struct PrimIdx
    /// The class of a primitive.
    /// Each attribute is the index of a vertex.
    struct PrimIdx{
        int a, b, c, d;
    };

    /// @param PrimIdxVertices
    /// The offsets for each vertex of each primitive that are used for the
    /// temporary storage during the computation of the normals on the GPU
    struct PrimIdxVertices{
        int ia, ib, ic ,id;

        PrimIdxVertices() { }

        PrimIdxVertices(int ia_, int ib_, int ic_, int id_) :
            ia(ia_), ib(ib_), ic(ic_), id(id_)
        {  }

        void print(){
            printf("%d %d %d %d\n",ia, ib, ic, id);
        }
    };

    //  ------------------------------------------------------------------------
    // End Inner structs
    //  ------------------------------------------------------------------------

    /// @warning this don't copy texture but only pointer to texture
    Mesh(const Mesh& m);

    /// Load a mesh from the abstract representation of our file loader
    Mesh(const Loader::Abs_mesh& mesh);

    ~Mesh();

    /// Check for data corruptions in the mesh  and exit programm if there is any.
    /// This is a debug function, if it triggers it is likely this class is bugged.
    void check_integrity();

    //  ------------------------------------------------------------------------
    /// @name Getter & Setters
    //  ------------------------------------------------------------------------

    /// Get the offset
    inline const Vec3_cu& get_offset() const{ return _offset; }

    /// Get the scale
    inline float get_scale() const { return _scale; }

    /// Get vertex no. i
    Vec3_cu get_vertex(int i) const {
        return Vec3_cu(_vert[3*i], _vert[3*i+1], _vert[3*i+2]);
    }

    /// Get the normal at vertex no. i
    /// @param n sometimes a vertex has multiples normals
    /// (either duplicated because of texture coordinates or different for the shading)
    /// the parameter is a way to fetch at the ith vertex the nth normal.
    /// @see get_nb_normals()
    Vec3_cu get_normal(int i, int n = 0) const;

    /// Get the normal at vertex no. i
    /// @return the mean normal at vertex 'i'
    /// (sometimes a vertex has multiples normals)
    /// @see get_normal()
    Vec3_cu get_mean_normal(int i) const;

    /// Get the offsets for primitive no. i
    PrimIdxVertices get_piv(int i) const {
        return PrimIdxVertices(_piv[4*i], _piv[4*i +1], _piv[4*i + 2],  _piv[4*i + 3]);
    }

    /// Get the index at i
    int get_tri (int i) const { return _tri [i]; }

    const int* get_tri_index () const { return _tri;  }

    const float* get_vertices() const { return _vert; }

    int get_edge(int i) const {
        assert(i < _nb_edges);
        return _edge_list[i];
    }

    int get_edge_offset(int i) const {
        assert(i < _nb_vert*2);
        return _edge_list_offsets[i];
    }

    int get_first_neighbor(int i) const {
        assert(i < _nb_vert);
        return _edge_list_offsets[i*2];
    }

    int get_num_neighbors(int i) const {
        assert(i < _nb_vert);
        return _edge_list_offsets[i*2 + 1];
    }

    int get_nb_vertices() const { return _nb_vert;                  }
    int get_nb_tri()      const { return _nb_tri;                   }
    int get_nb_faces()    const { return _nb_tri;                   }
    int get_nb_edges()    const { return _nb_edges;                 }

    /// @return false if the ith vertex belongs to at least one primitive
    bool is_disconnect(int i) const { return !_is_connected[i]; }

    /// Is the ith vertex on the mesh boundary
    bool is_vert_on_side(int i) const { return _is_side[i]; }

private:

    //  ------------------------------------------------------------------------
    /// @name Class tools
    //  ------------------------------------------------------------------------

    /// Free memory for every attributes even std::vectors
    void free_mesh_data();

    /// Allocate and compute the offsets for the computation of the normals
    /// on the GPU
    void compute_piv();

    /// Compute the normals on CPU
    void compute_normals();

    /// Compute the list of the mesh edges
    /// updates 'edge_list' and 'edge_list_offsets'
    void compute_edges();

    // Load _edge_list and _edge_list_offsets, given an array of neighbors for each vertex.
    void load_edges(const std::vector<std::vector<int> > &neighborhood_list);

    // Mesh edges computation tool functions.
    //{
    /// given a triangle 'index_tri' and one of its vertex index 'current_vert'
    /// return the pair corresponding to the vertex index opposite to 'current_vert'
    std::pair<int, int> pair_from_tri(int index_tri, int current_vert);

    //  ------------------------------------------------------------------------
    /// @name Attributes
    //  ------------------------------------------------------------------------

    //  ------------------------------------------------------------------------
    /// @name global properties
    //  ------------------------------------------------------------------------

    bool _is_initialized;    ///< is the mesh renderable (means every attributes is filled correctly)
    bool _has_normals;       ///< do the mesh has normals loaded

    Vec3_cu _offset;
    float  _scale;
public:
    int _max_faces_per_vertex;
private:
    //  ------------------------------------------------------------------------
    /// @name Packed data
    //  ------------------------------------------------------------------------
    int _nb_vert;
    int _nb_tri;   ///< number of triangle faces (same for packed and unpacked)
    int _nb_edges; ///< nb elt in 'edge_list'

    // TODO: use a structure to hold vertices properties.
    float* _vert;        ///< Vertex position list [V0x V0y V0z V1x V1y V1z ...]

    bool* _is_connected; ///< Does the ith vertex belongs to a tri
    std::vector<bool> _is_side;      ///< Does the ith vertex belongs to a mesh boundary

    int* _tri;           ///< triangle index

    /// ?
    int* _piv;

    /// list of neighbours of each vertex
    /// @see edge_list_offsets
    int* _edge_list;

    /// 1st index and number of neighbours of each vertex in the edge_list
    /// array size is twice the number of vertices (2*nb_vert)
    /// @see edge_list
    int* _edge_list_offsets;

    /// List of vertex on the side of the mesh
    std::vector<int> _on_side_verts;

    //  ------------------------------------------------------------------------
    /// @name unpacked data
    //  ------------------------------------------------------------------------

    float* _normals;     ///< Normal direction list [N0x N0y N0z N1x N1y N1z ...]

    /// size of the vbo for the rendering. '_normals' and '_tangents' size are
    /// 3*size_unpacked_vert_array and '_tex_coords' is 2*size_unpacked_vert_array
    int _size_unpacked_vert_array;

    /// Mapping between the packed array of vertices 'vert' and the unpacked
    /// array of vertices 'vbo'. because vertices have multiple texture
    /// coordinates we have to duplicate them. we do that for the rendering
    /// by duplicating the original data in 'vert' into the vbos.
    /// packed_vert_map[packed_vert_idx] = mapping to unpacked.
    /// size of 'packed_vert_map' equals 'nb_vert'
    Packed_data* _packed_vert_map;
};

#endif // MESH_HPP__



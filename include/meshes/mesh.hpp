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

#include "glbuffer_object.hpp"
#include "shader.hpp"
#include "vec3_cu.hpp"

// FORWARD DEFINTIONS  ---------------------------------------------------------
class GlTex2D;

namespace Loader{
    struct Abs_mesh;
}

struct Animesh;
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

    friend struct Animesh;

    //  ------------------------------------------------------------------------
    /// @name Inner structs
    //  ------------------------------------------------------------------------

    /** @struct Packed_data
    */
    struct Packed_data{
        /// index of the data in the unpacked array
        int idx_data_unpacked;
        /// number of occurences consecutive to idx_data_unpacked in the
        /// unpacked array
        int nb_ocurrence;
    };

    struct Tex_coords{
        Tex_coords() : u(0.f), v(0.f) { }
        Tex_coords(float u_, float v_) : u(u_), v(v_) { }
        float u,v;
    };

    struct Tri_idx {
        int a, b, c;
    };

    struct Quad_idx {
        int a, b, c, d;
    };

    /// @struct PrimIdx
    /// The class of a primitive (triangle or quad).
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

    struct Mat_grp{
        int starting_idx; ///< starting face index to apply the material
        int nb_face;      ///< (starting_idx+nb_face) is the ending face index
        int mat_idx;      ///< material index to apply to the set of faces
    };

    //  ------------------------------------------------------------------------
    // End Inner structs
    //  ------------------------------------------------------------------------

    /// @warning this don't copy texture but only pointer to texture
    Mesh(const Mesh& m);

    Mesh();

    /// Build mesh from triangle index and vertex position
    Mesh(const std::vector<int>& tri, const std::vector<float>& vert);

    ~Mesh();

    /// Check for data corruptions in the mesh  and exit programm if there is any.
    /// This is a debug function, if it triggers it is likely this class is bugged.
    void check_integrity();


    //  ------------------------------------------------------------------------
    /// @name Import/export
    //  ------------------------------------------------------------------------

    /// Create a mesh from an OFF file
    Mesh(const char* filename);

    /// @param filename      path and name for the file to be writen
    /// @param invert_index  wether the quads and triangles index are to be
    /// inverted (i.e write in clockwise order or counter clockwise)
    void export_off(const char* filename, bool invert_index = false) const;

    /// Load a mesh from the abstract representation of our file loader
    void load(const Loader::Abs_mesh& mesh, const std::string& mesh_path);

    /// Save a mesh into the abstract representation
    void save(Loader::Abs_mesh& mesh);

    //  ------------------------------------------------------------------------
    /// @name Drawing the mesh
    //  ------------------------------------------------------------------------

    //  ------------------------------------------------------------------------
    /// @name Surface deformations
    //  ------------------------------------------------------------------------

    /// Center about the center of gravity and resize the mesh to the given
    /// length 'size'
    void center_and_resize(float size);

    /// Smoothen the mesh (make the surface less rough)
    void smoothen_mesh(float smooth_factor,
                       int nb_iter,
                       int nb_min_neighbours = 0);

    /// Diffuse some vertex attributes along the mesh.
    /// Each vertex is associated to an attribute defined by the array
    /// "vertices_attributes[]". For each vertex the mean sum of its first ring
    /// of neighborhood is computed, the operation is repeated "nb_iter" times.
    /// @return The new vertices attributes in "vertices_attributes[]" array
    void diffuse_along_mesh(float* vertices_attributes, int nb_iter) const;

    /// Diffuse some vertex attributes along the mesh. see method above.
    /// this method is the same but does not change attributes values which
    /// equals to "locked_value"
    void diffuse_along_mesh(float* vertices_attributes,
                            float locked_value,
                            int nb_iter) const;

    /// Add noise to the mesh
    void add_noise(int fq, float amp);

    //  ------------------------------------------------------------------------
    /// @name Getter & Setters
    //  ------------------------------------------------------------------------

    /// Set the color of the ith vertices when the mesh points are displayed
    /// @warning slow method for large data don't use this use directly the
    /// buffer object
    void set_point_color_bo(int i, float r, float g, float b, float a);

    /// Set the color_bo attribute to the given rgba color.
    /// @see color_bo
    void set_color_bo(float r, float g, float b, float a);

    /// Set the point_color_bo attribute to the given rgba color.
    /// @see point_color_bo
    void set_point_color_bo(float r, float g, float b, float a);

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

    /// Get the texture coordinates at vertex no. i
    /// @param n sometimes a vertex has multiples texture coordinates
    /// the parameter 'n' is a way to fetch at the ith vertex the nth normal.
    /// @see get_nb_tex_coords()
    Tex_coords get_tex_coords(int i, int n = 0) const;

    /// Get the offsets for primitive no. i
    PrimIdxVertices get_piv(int i) const {
        return PrimIdxVertices(_piv[4*i], _piv[4*i +1], _piv[4*i + 2],  _piv[4*i + 3]);
    }

    /// Get the index at i
    int get_tri (int i) const { return _tri [i]; }
    int get_quad(int i) const { return _quad[i]; }

    const int* get_tri_index () const { return _tri;  }
    const int* get_quad_index() const { return _quad; }

    const float* get_vertices() const { return _vert; }

    /// List of vertex indices which are connected to not manifold triangles
    const std::vector<int>& get_not_manifold_list() const { return _not_manifold_verts; }

    /// List of vertices on a side of the mesh
    const std::vector<int>& get_on_side_list() const { return _on_side_verts; }

    const Packed_data* get_packed_vert_map() const { return _packed_vert_map; }

    int get_edge(int i) const {
        assert(i < _nb_edges);
        return _edge_list[i];
    }

    int get_edge_offset(int i) const {
        assert(i < _nb_vert*2);
        return _edge_list_offsets[i];
    }

    int get_nb_vertices() const { return _nb_vert;                  }
    int get_nb_tri()      const { return _nb_tri;                   }
    int get_nb_quad()     const { return _nb_quad;                  }
    int get_nb_faces()    const { return _nb_tri + _nb_quad;        }
    int get_nb_edges()    const { return _nb_edges;                 }
    int get_vbos_size()   const { return _size_unpacked_vert_array; }

    /// number of normals associated to the ith vertex
    int get_nb_normals(int i) const { return _packed_vert_map[i].nb_ocurrence; }
    /// number of texture coordinates associated to the ith vertex
    int get_nb_tex_coords(int i) const { return _packed_vert_map[i].nb_ocurrence; }

    /// @return false if the ith vertex belongs to at least one primitive
    /// i.e triangle or quad
    bool is_disconnect(int i) const { return !_is_connected[i]; }

    /// Is the ith vertex on the mesh boundary
    bool is_vert_on_side(int i) const { return _is_side[i]; }

    bool is_closed()      const { return _is_closed;      }
    bool has_tex_coords() const { return _has_tex_coords; }
    bool has_normals()    const { return _has_normals;    }
    bool is_manifold()    const { return _is_manifold;    }

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

    void compute_tangents();

    /// For each vertex compute the list of faces it belongs to and stores it
    /// in the attributes 'tri_list_per_vert' and 'quad_list_per_vert'.
    void compute_face_index();

    /// Compute the list of the mesh edges
    /// updates 'edge_list' and 'edge_list_offsets'
    void compute_edges();

    /// initialize all the vbos with their corresponding CPU array attributes
    void init_vbos();

    /// updates the vbo with the vertex position in 'vert'.
    /// @warning slow method it is done on CPU
    void update_unpacked_vert();

    // Mesh edges computation tool functions.
    //{
    /// given a triangle 'index_tri' and one of its vertex index 'current_vert'
    /// return the pair corresponding to the vertex index opposite to 'current_vert'
    std::pair<int, int> pair_from_tri(int index_tri, int current_vert);

    /// Same as pair_from_tri() but with quads. 'n' is between [0 1] and tells
    /// if the first or the seccond pair of vertex is to be chosed.
    std::pair<int, int> pair_from_quad(int index_quad, int current_vert, int n);
    //}


    //  ------------------------------------------------------------------------
    /// @name Attributes
    //  ------------------------------------------------------------------------

    //  ------------------------------------------------------------------------
    /// @name global properties
    //  ------------------------------------------------------------------------

    bool _is_initialized;    ///< is the mesh renderable (means every attributes is filled correctly)
    bool _is_closed;         ///< is the mesh closed
    bool _has_tex_coords;    ///< do the mesh has texture coordinates loaded
    bool _has_normals;       ///< do the mesh has normals loaded

    /// When false it is sure the mesh is not 2-manifold. But true does not
    /// ensure mesh is 2-manifold (for instance self intersection are not detected
    /// and when is_closed == false, is_manifold remains true). Mainly
    /// topological defects are detected with is_manifold == false.
    bool _is_manifold;

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
    int _nb_quad;  ///< number of quad faces (same for packed and unpacked)
    int _nb_edges; ///< nb elt in 'edge_list'

    // TODO: use a structure to hold vertices properties.
    float* _vert;        ///< Vertex position list [V0x V0y V0z V1x V1y V1z ...]

    bool* _is_connected; ///< Does the ith vertex belongs to a tri or a quad
    bool* _is_side;      ///< Does the ith vertex belongs to a mesh boundary

    int* _tri;           ///< triangle index
    int* _quad;          ///< quad index

    /// list of triangles index connected to a vertices.
    /// tri_list_per_vert[index_vert][nb_connected_triangles] = index triangle in
    /// attribute 'tri'
    std::vector<std::vector<int> > _tri_list_per_vert;
    /// list of quads index connected to a vertices.
    /// quad_list_per_vert[index_vert][nb_connected_quads] = index quad in
    /// attribute 'quad'
    std::vector<std::vector<int> > _quad_list_per_vert;

    /// ?
    int* _piv;

    /// list of neighbours of each vertex
    /// N.B : quads are triangulated before creating this list of neighbours
    /// @see edge_list_offsets
    int* _edge_list;

    /// 1st index and number of neighbours of each vertex in the edge_list
    /// array size is twice the number of vertices (2*nb_vert)
    /// @see edge_list
    int* _edge_list_offsets;

    /// List of packed vertex index which presents topological defects.
    std::vector<int> _not_manifold_verts;
    /// List of vertex on the side of the mesh
    std::vector<int> _on_side_verts;

    //  ------------------------------------------------------------------------
    /// @name unpacked data
    //  ------------------------------------------------------------------------

    float* _normals;     ///< Normal direction list [N0x N0y N0z N1x N1y N1z ...]
    float* _tangents;    ///< Tangent direction list [T0x T0y T0z T1x T1y T1z ...]
    float* _tex_coords;  ///< Texture coordinates list [T0u T0v T1u T1v ...]

    /// size of the vbo for the rendering. '_normals' and '_tangents' size are
    /// 3*size_unpacked_vert_array and '_tex_coords' is 2*size_unpacked_vert_array
    int _size_unpacked_vert_array;

    int* _unpacked_tri;  ///< unpacked triangle index ( size == nb_tri)
    int* _unpacked_quad; ///< unpacked quad index (size == nb_quad)

    /// list of triangles index connected to a vertices.
    /// unpacked_tri_list_per_vert[index_vert][nb_connected_triangles] = index triangle in
    /// attribute 'unpacked_tri'
    std::vector<std::vector<int> > _unpacked_tri_list_per_vert;
    /// list of quads index connected to a vertices.
    /// unpacked_quad_list_per_vert[index_vert][nb_connected_quads] = index quad in
    /// attribute 'unpacked_quad'
    std::vector<std::vector<int> > _unpacked_quad_list_per_vert;

    /// Mapping between the packed array of vertices 'vert' and the unpacked
    /// array of vertices 'vbo'. because vertices have multiple texture
    /// coordinates we have to duplicate them. we do that for the rendering
    /// by duplicating the original data in 'vert' into the vbos.
    /// packed_vert_map[packed_vert_idx] = mapping to unpacked.
    /// size of 'packed_vert_map' equals 'nb_vert'
    Packed_data* _packed_vert_map;

public:
    /// @name Buffer object data
    /// @brief size of these buffers are 'size_unpacked_vert_array'
    /// @{
    GlBuffer_obj _vbo;
    GlBuffer_obj _normals_bo;
    GlBuffer_obj _tangents_bo;
    GlBuffer_obj _color_bo;
    GlBuffer_obj _tex_bo;
    /// color of mesh's points when points are displayed
    GlBuffer_obj _point_color_bo;
    /// @}

    /// @name Buffer object index
    /// @{
    /// Size of this buffer is 'nb_tri'
    GlBuffer_obj _index_bo_tri;
    /// Size of this buffer is 'nb_quad'
    GlBuffer_obj _index_bo_quad;
    /// Size of this buffer is 'nb_vert'
    GlBuffer_obj _index_bo_point;
    /// @}
};

#endif // MESH_HPP__



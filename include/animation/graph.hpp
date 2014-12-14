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
#ifndef GRAPH_HPP__
#define GRAPH_HPP__

#include "point_cu.hpp"
#include <vector>

class Camera;

/** @brief The class of a graph.
    A graph can be used to design a skeleton from scratch with push_vertex(),
    push_edge() and remove_vertex(). The result can be saved with
    save_to_file(). One can also load a ".skel" file with the method
    load_from_file().

    Drawing the graph is done with draw()

    Selection can be done through get_window_nearest() wich returns the nearest
    vertex from the mouse
*/
struct Graph {

  /// An edge of the graph is defined by the ID of the two vertices it joins.
  struct Edge{
    int a, b;
    inline Edge(){}
    inline Edge(int x, int y) : a(x), b(y) {}

    bool operator==(const Edge& e) const {
        return (e.a == a && e.b == b) || (e.a == b && e.b == a);
    }
  };

  Graph() : _offset( Vec3_cu::zero() ), _scale(1.f) { }
  Graph(const Vec3_cu& off, float s) : _offset(off), _scale(s) {}

  /// @return true if there is cycles
  bool is_cycles(int root) const;

  //----------------------------------------------------------------------------
  /// @name Graph editing
  //----------------------------------------------------------------------------

  /// Push a vertex in the vertices list
  int push_vertex(const Vec3_cu& v);

  /// Push an edge in the edges list
  int push_edge(const Edge& e);

  /// Remove a vertex from the graph
  void remove_vertex(int i);

  /// Clear all the graph vertices and edges
  void clear();

  //----------------------------------------------------------------------------
  /// @name Graph gui
  //----------------------------------------------------------------------------

  /// Get the nearest vertex from window position (x,y).
  /// This is used to perfom vertex selection
  /// @param[out] dist the distance between the mouse and the screen projection
  /// of the selected vertex
  /// @return the selected vertex id
  int get_window_nearest(float x, float y, float& dist);

  /// Draw the graph with raster
  void draw(const Camera& cam, int current_vertex = -1) const;

  /// Try to find the
  // void mirror(int i);

  //----------------------------------------------------------------------------
  /// @name Import/export
  //----------------------------------------------------------------------------

  /// Save the graph to some file
  void save_to_file(const char* filename) const;

  /// Load the graph from some file and erase the previous graph
  void load_from_file(const char* filename);

  //----------------------------------------------------------------------------
  /// @name Getters & Setters
  //----------------------------------------------------------------------------

  /// Get a vertex from its ID
  inline const Vec3_cu& get_vertex(int i) const { return _vertices[i];  }
  inline       Vec3_cu& get_vertex(int i)       { return _vertices[i];  }

  void set_vertex(int i, Vec3_cu v) { _vertices[i] = v;  }

  /// Scale and offset the graph
  void set_offset_scale(const Vec3_cu& _offset, float _scale);

  /// Return the number of vertices
  inline int nb_vertices() const { return _vertices.size(); }

  /// Return the number of edges
  inline int nb_edges() const { return _edges.size(); }

  //----------------------------------------------------------------------------
  /// @name Attributes
  //----------------------------------------------------------------------------

  // TODO: must be private add accessors etc.
  // Use std::vectors
  // Offset and scale can be applied then through accessors
  /// List of nodes positions
  std::vector<Vec3_cu> _vertices;
  /// List of edges
  std::vector<Edge> _edges;
  /// List of nodes neighborhoods _neighs[node_id][nb_neigh] = neighs_id
  std::vector< std::vector<int> > _neighs;

  Vec3_cu _offset;
  float   _scale;
};

#endif // GRAPH_HPP__

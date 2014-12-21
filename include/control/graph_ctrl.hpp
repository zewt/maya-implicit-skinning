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
#ifndef GRAPH_CTRL_HPP_
#define GRAPH_CTRL_HPP_

/** @brief graph controller
    This class is an utility to control the graph.


*/

#include "vec3_cu.hpp"

class Graph_ctrl {
public:

    Graph_ctrl() :
        _selected_node(-1)
    {
    }

    void save_to_file(const char* filename) const;
    void load_from_file(const char* filename) const;

    bool is_loaded();

    /// Push a vertex in the vertices list
    int push_vertex(const Vec3_cu& v);

    void remove(int i);

    /// Push an edge in the edges list
    int push_edge(int v1, int v2);

    Vec3_cu get_vertex(int id);

    void set_vertex(int id, Vec3_cu v);

    /// Try to center the node inside the mesh
    void center_vertex(int id);

    int  get_selected_node()     { return _selected_node; }
    void set_selected_node(int n){ _selected_node = n;    }
    void reset_selection()       { _selected_node = -1;   }

private:
    int _selected_node;
};



#endif // GRAPH_CTRL_HPP_

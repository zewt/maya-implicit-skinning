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
#include "graph_ctrl.hpp"

#include <iostream>

#include "graph.hpp"
#include "cuda_globals.hpp"
#include "globals.hpp"

// -----------------------------------------------------------------------------

// XXX: unused, we don't save the skeleton to disk
void Graph_ctrl::save_to_file(const char* filename) const
{
    g_graph->save_to_file(filename);
}

// -----------------------------------------------------------------------------

void Graph_ctrl::load_from_file(const char* filename) const{
    g_graph->load_from_file(filename);
    std::cout << "Loading from file: " << filename << std::endl;
    g_graph->set_offset_scale(g_mesh->get_offset(), g_mesh->get_scale());
}

// -----------------------------------------------------------------------------

bool Graph_ctrl::is_loaded(){
    return g_graph->nb_vertices() != 0;
}

// -----------------------------------------------------------------------------

int Graph_ctrl::push_vertex(const Vec3_cu &v)
{
    return g_graph->push_vertex(Vec3_cu(v.x, v.y, v.z));
}

// -----------------------------------------------------------------------------

void Graph_ctrl::remove(int i)
{
    _selected_node = -1;
    return g_graph->remove_vertex(i);
}

// -----------------------------------------------------------------------------

int Graph_ctrl::push_edge(int v1, int v2)
{
    return g_graph->push_edge(Graph::Edge(v1, v2));
}

// -----------------------------------------------------------------------------

Vec3_cu Graph_ctrl::get_vertex(int id)
{
    Vec3_cu v = g_graph->get_vertex(id);
    return Vec3_cu(v.x, v.y, v.z);
}

// -----------------------------------------------------------------------------

void Graph_ctrl::set_vertex(int id, Vec3_cu v){
    if( is_loaded() )
        g_graph->set_vertex( id, Vec3_cu(v.x, v.y, v.z) );
}

// -----------------------------------------------------------------------------

void Graph_ctrl::center_vertex(int )
{
}

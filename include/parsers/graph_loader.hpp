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
#ifndef GRAPH_LOADER_HPP__
#define GRAPH_LOADER_HPP__

#include "loader.hpp"
#include "graph.hpp"

// =============================================================================
namespace Graph_loader {
// =============================================================================

class Graph_file : public Loader::Base_loader {
public:
    Graph_file(const std::string& file_name) : Base_loader( file_name )
    { import_file(file_name);  }

    /// The loader type
    Loader::Loader_t type() const { return Loader::SKEL; }

    bool import_file(const std::string& file_path){
        _graph.clear();
        _graph.load_from_file( file_path.c_str() );
        return true;
    }

    bool export_file(const std::string& file_path){
        _graph.save_to_file( file_path.c_str() );
        return true;
    }

    /// @return parsed animations or NULL.
    void get_anims(std::vector<Loader::Base_anim_eval*>& anims) const { anims.clear(); }

    /// transform internal representation into generic representation
    /// which are the same here.
    void get_graph(Graph& graph) const { graph = _graph; }

private:
    Graph _graph;
};

}// END Graph_loader ============================================================

#endif // GRAPH_LOADER_HPP__

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
#ifndef GRID_HPP
#define GRID_HPP

#include "idx3_cu.hpp"
#include "vec3i_cu.hpp"
#include "tree.hpp"
#include <vector>
#include <deque>
#include <list>
#include <set>

// =============================================================================
namespace Skeleton_env {
// =============================================================================

struct Grid {

    Grid(const Tree* tree);

    /// Update Grid's datas.
    /// Build the grid cells given the current associated tree and states.
    /// Call this each time the tree data/position changes
    void build_grid();

    //--------------------------------------------------------------------------
    /// @name Accessors
    //--------------------------------------------------------------------------

    /// Change the resolution x=y=z=res of the grid
    void set_res(int res);

    /// resolution of the grid res^3 = x.y.z
    int res() const;

    /// 3d position to grid integer index
    Vec3i_cu index_cell( Vec3_cu p ) const;

    /// Total size in the scene of the grid;
    Vec3_cu fsize_grid() const;

    /// x, y and z cell size of the grid
    Vec3_cu cell_size() const;

    BBox_cu bbox() const { return _pos; }

    //--------------------------------------------------------------------------
    /// @name Datas
    //--------------------------------------------------------------------------

    /// 3D grid stored linearly of size _res*_res*_res.
    /// Each cell contains a list of bones that intersects it
    std::vector< std::list<Bone::Id> > _grid_cells;

    /// Indices of the cells empty not empty.
    std::set<int> _filled_cells;

private:

    //--------------------------------------------------------------------------
    /// @name Class tools
    //--------------------------------------------------------------------------

    /// clear _filled_cells and list of bones for each cell in _grid_cells
    void reset_grid();

    /// Fill the bone list in cells with the current tree '_tree'
    void add_tree();

    /// Add a bone to the cell the bounding box intersects
    void add_bone( Bone::Id bid );

    //--------------------------------------------------------------------------
    /// @name Attributes
    //--------------------------------------------------------------------------

    /// Associated tree to the grid
    const Tree* _tree;

    /// x, y resolutions of the grid
    int _res;

    ///  position and length of the grid represented with a bbox.
    BBox_cu _pos;
};


}// NAMESPACE END Skeleton_env  ================================================

#endif // GRID_HPP

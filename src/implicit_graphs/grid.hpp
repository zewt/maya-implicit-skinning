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

    Grid(const Tree* tree, int res=-1);

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
    std::vector< std::vector<Bone::Id> > _grid_cells;

    /// Indices of the cells empty not empty.
    std::vector<bool> _filled_cells;

private:

    //--------------------------------------------------------------------------
    /// @name Class tools
    //--------------------------------------------------------------------------

    /// clear _filled_cells and list of bones for each cell in _grid_cells
    void reset_grid();

    /// Add a bone to the cell the bounding box intersects
    void add_bone(const Bone *bone);

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

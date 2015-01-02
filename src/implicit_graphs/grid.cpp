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
#include "grid.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

Grid::Grid(const Tree* tree) :
    _tree(tree),
    _res(20)

{
    _grid_cells.resize( _res * _res * _res);
    build_grid();
}

// -----------------------------------------------------------------------------

void Grid::set_res(int res)
{
    assert( res > 0);
    _grid_cells.clear();
    _res = res;
    _grid_cells.resize( res * res * res);
    build_grid();
}

// -----------------------------------------------------------------------------

int Grid::res() const { return _res; }

// -----------------------------------------------------------------------------

Vec3i_cu Grid::index_cell( Vec3_cu p ) const
{
    return _pos.index_grid_cell(Vec3i_cu(res(), res(), res()), p);
}

// -----------------------------------------------------------------------------

Vec3_cu Grid::fsize_grid() const {
    assert(_pos.is_valid());
    return _pos.pmax - _pos.pmin;
}

// -----------------------------------------------------------------------------

Vec3_cu Grid::cell_size() const { return fsize_grid() / (float)_res;}

// -----------------------------------------------------------------------------

void Grid::build_grid()
{
    reset_grid();
    _pos = _tree->bbox();
    // Slightly enlarge bbox to avoid being perfectly aligned with bone bbox
    const float e = 0.00001f;
    const Vec3_cu eps(e, e, e);
    _pos.pmin = _pos.pmin - eps;
    _pos.pmax = _pos.pmax + eps;
    // Add tree if bbox is large enough
    if( _pos.is_valid( ) )
        add_tree();
}

// -----------------------------------------------------------------------------

void Grid::reset_grid()
{
    for( unsigned i = 0; i < _grid_cells.size(); ++i)
        _grid_cells[i].clear();
    _filled_cells.clear();
}

// -----------------------------------------------------------------------------

void Grid::add_tree()
{
    for(int i = 0; i < _tree->bone_size(); ++i) //FIXME: add from root bone to leaf with rec lookup
    {
        // Ignore root joints.
        if( _tree->parent(i) == -1 ) continue;
        add_bone( _tree->bone(i)->get_bone_id() );
    }
}

// -----------------------------------------------------------------------------

void Grid::add_bone( Bone::Id bid )
{
    const Bone* bone = _tree->bone( bid );
    if( bone->get_type() == EBone::SSD)
        return;

    if( bone->get_type() == EBone::HRBF){
        const Bone_hrbf* bone_hrbf = ((Bone_hrbf*)bone);
        if( bone_hrbf->get_hrbf().empty() )
            return;
    }

    // Lookup every cell inside the bbox and add the bone id to these cells.
    BBox_cu bb = bone->get_bbox();
    if( !bb.is_valid() ) return;

    Vec3i_cu min_idx = index_cell( bb.pmin ).clamp(0, _res-1);
    Vec3i_cu max_idx = index_cell( bb.pmax ).clamp(0, _res-1);

    Vec3i_cu sub_size = max_idx - min_idx + 1;
    assert(sub_size.product() >= 0);


    Vec3i_cu grid_size(_res, _res, _res);
    Idx3_cu offset(grid_size, min_idx);
    for(Idx3_cu idx(sub_size, 0); idx.is_in(); ++idx)
    {
        int i = (offset + idx.to_vec3i()).to_linear();
        assert(i < grid_size.product() );
        assert(i >= 0);
        _filled_cells.insert( i );
        _grid_cells[i].push_back( bid );
    }

    if( sub_size.product() == 0)
    {
        int i = offset.to_linear();
        assert( offset.is_in() );
        _filled_cells.insert( i );
        _grid_cells[i].push_back( bid );
    }
}

}// NAMESPACE END Skeleton_env  ================================================

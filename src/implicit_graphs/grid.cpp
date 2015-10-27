#include "grid.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

Grid::Grid(const Tree* tree, int res) :
    _tree(tree)
{
    if(res == -1)
        res = 20;
    _res = res;
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

    // If the bounding box is too small, don't do anything.
    if(!_pos.is_valid())
        return;

    for(auto bone: _tree->bones())
        add_bone(bone);
}

void Grid::reset_grid()
{
    int total_bones = _tree->bones().size();
    for( unsigned i = 0; i < _grid_cells.size(); ++i) {
        _grid_cells[i].clear();
        _grid_cells[i].reserve(total_bones);
    }
    _filled_cells.assign(_grid_cells.size(), false);
}

void Grid::add_bone(const Bone *bone)
{
    if( bone->get_type() == EBone::SSD)
        return;

    if( bone->get_type() == EBone::HRBF){
        if( bone->get_hrbf().empty() )
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
        _filled_cells[i] = true;
        _grid_cells[i].push_back( bone->get_bone_id() );
    }

    if( sub_size.product() == 0)
    {
        int i = offset.to_linear();
        assert( offset.is_in() );
        _filled_cells[i] = true;
        _grid_cells[i].push_back(bone->get_bone_id());
    }
}

}// NAMESPACE END Skeleton_env  ================================================

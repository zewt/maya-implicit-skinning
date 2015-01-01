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
#include "tree.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

Tree::Tree(int root,
           const std::vector<const Bone*>& bones,
           const std::vector<int>& parents) :
    _root( root ),
    _bones( bones.begin(), bones.end() ),
    _parents( parents )
{
    _datas.resize( bones.size() );
    _sons.resize( bones.size() );
    // Build sons adjency :
    for(unsigned i = 0; i < _sons.size(); ++i) {
        int pt = _parents[i];
        if(pt > -1) _sons[pt].push_back( i );
    }
}

// -----------------------------------------------------------------------------

/// Compute the axis aligned bounding box of the tree
BBox_cu Tree::bbox() const
{
    BBox_cu res;
    for(int i = 0; i < bone_size(); ++i)
        res = res.bbox_union( _bones[i]->get_bbox() );

    return res;
}


}// NAMESPACE END Skeleton_env  ================================================

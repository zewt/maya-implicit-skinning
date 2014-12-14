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
#ifndef BBOX2I_CU_HPP
#define BBOX2I_CU_HPP

#include <float.h>
#include <vector>
#include <cassert>
#include <climits>

#include "cuda_compiler_interop.hpp"
#include "ray_cu.hpp"
#include "transfo.hpp"
#include "vec2i_cu.hpp"

/**
  @struct BBox2i_cu
  @brief Representation with two integer 2d points of an axis aligned bounded box
*/
struct BBox2i_cu {

    /// using infinity allows maximum and minimum to take the other value,
    /// always. The union of two empty bboxes is still an empty bbox,
    /// and the intersection of an empty bbox with another bbox
    /// is still empty finally a point will be considered outside the bbox
    IF_CUDA_DEVICE_HOST
    inline BBox2i_cu():
        pmin(Vec2i_cu(INT_MAX, INT_MAX)),
        pmax(Vec2i_cu(INT_MIN, INT_MIN))
    {    }

    IF_CUDA_DEVICE_HOST
    inline BBox2i_cu(const Vec2i_cu& a, const Vec2i_cu& b):
        pmin(Vec2i_cu(min(a.x, b.x), min(a.y, b.y))),
        pmax(Vec2i_cu(max(a.x, b.x), max(a.y, b.y)))
    {    }

    IF_CUDA_DEVICE_HOST
    inline BBox2i_cu(int xmin, int ymin,
                     int xmax, int ymax):
        pmin(Vec2i_cu(xmin, ymin)),
        pmax(Vec2i_cu(xmax, ymax))
    {   }

    IF_CUDA_DEVICE_HOST
    inline bool inside(const Vec2i_cu& p) const;

    IF_CUDA_DEVICE_HOST
    inline BBox2i_cu bbox_union(const BBox2i_cu& bb) const;

    IF_CUDA_DEVICE_HOST
    inline BBox2i_cu bbox_isect(const BBox2i_cu& bb) const;

    IF_CUDA_DEVICE_HOST
    inline void add_point(const Vec2i_cu& p);

    /// Get x, y and z lenghts of the bouding box
    IF_CUDA_DEVICE_HOST
    inline Vec2i_cu lengths() const;

    /// A valid bbox as length stricly superior to zero
    IF_CUDA_DEVICE_HOST
    inline bool is_valid() const {
        return (pmax.x - pmin.x) > 0 &&
               (pmax.y - pmin.y) > 0;
    }

    Vec2i_cu pmin;
    Vec2i_cu pmax;
};

IF_CUDA_DEVICE_HOST inline
bool BBox2i_cu::inside(const Vec2i_cu& p) const {
    return (p.x >= pmin.x) & (p.y >= pmin.y) &
           (p.x <= pmax.x) & (p.y <= pmax.y);
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
BBox2i_cu BBox2i_cu::bbox_union(const BBox2i_cu& bb) const {
    return BBox2i_cu(min(pmin.x, bb.pmin.x), min(pmin.y, bb.pmin.y),
                     max(pmax.x, bb.pmax.x), max(pmax.y, bb.pmax.y));
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
BBox2i_cu BBox2i_cu::bbox_isect(const BBox2i_cu& bb) const {

    BBox2i_cu res =
            BBox2i_cu(max(pmin.x, bb.pmin.x), max(pmin.y, bb.pmin.y),
                      min(pmax.x, bb.pmax.x), min(pmax.y, bb.pmax.y));

    if( (res.pmin.x > res.pmax.x) | (res.pmin.y > res.pmax.y) )
        res = BBox2i_cu();

    return res;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
void BBox2i_cu::add_point(const Vec2i_cu& p) {
    pmin.x = min(p.x, pmin.x);
    pmin.y = min(p.y, pmin.y);
    pmax.x = max(p.x, pmax.x);
    pmax.y = max(p.y, pmax.y);
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
Vec2i_cu BBox2i_cu::lengths() const {
    return pmax - pmin;
}


#endif // BBOX2I_CU_HPP

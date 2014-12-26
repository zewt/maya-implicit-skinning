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
#ifndef BBOX_CU_HPP__
#define BBOX_CU_HPP__

#include <float.h>
#include <vector>
#include <cassert>
#include <climits>

#include "cuda_compiler_interop.hpp"
#include "ray_cu.hpp"
#include "transfo.hpp"
#include "vec2_cu.hpp"
#include "vec3i_cu.hpp"

// =============================================================================

/**
 * @struct BBox_cu
 * @brief Representation with two 3d points of an axis aligned bounded box
 * @note code compatible host/device/gcc/nvcc
 *
*/
struct BBox_cu{

    /// using infinity allows maximum and minimum to take the other value,
    /// always. The union of two empty bboxes is still an empty bbox,
    /// and the intersection of an empty bbox with another bbox
    /// is still empty finally a point will be considered outside the bbox
    IF_CUDA_DEVICE_HOST
    inline BBox_cu():
        pmin(Point_cu( FLT_MAX,  FLT_MAX,  FLT_MAX)),
        pmax(Point_cu(-FLT_MAX, -FLT_MAX, -FLT_MAX))
    {
    }

    IF_CUDA_DEVICE_HOST
    inline BBox_cu(const Point_cu& a, const Point_cu& b):
        pmin(Point_cu(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z))),
        pmax(Point_cu(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)))
    {    }

    IF_CUDA_DEVICE_HOST
    inline BBox_cu(float xmin, float ymin, float zmin,
                   float xmax, float ymax, float zmax):
        pmin(Point_cu(xmin, ymin, zmin)),
        pmax(Point_cu(xmax, ymax, zmax))
    {   }

    IF_CUDA_DEVICE_HOST
    inline bool inside(const Point_cu& p) const;


    /// Does the given ray intersect the bbox ? takes into account the ray's
    /// active segment. For instance, if the ray active segment [a,b] is inside
    /// the bbox, the resulting coordinates are [a,b], if the segment [a,b] is
    /// not included in the bbox, no intersection is detected, if [a,b] is
    /// partially in the bbox (let's say that ray(b) is inside the bbox and
    /// ray(a) is outside), then t0 = t_entry, t1 = b, and so on.
    /// @param ray : ray we want to intersect the bbox with
    /// @param tmin : contains the beginning of the ray segment and
    /// will contain the lower value of t for which it intersects
    /// @param tmax : contain the end of the ray segment and
    /// will contain the greater value of t for which it intersects
    /// @return true if it intersects.
    IF_CUDA_DEVICE_HOST inline
    bool isect(const Ray_cu& ray, float& tmin, float& tmax) const;

    IF_CUDA_DEVICE_HOST
    inline BBox_cu bbox_union(const BBox_cu& bb) const;

    IF_CUDA_DEVICE_HOST
    inline BBox_cu bbox_isect(const BBox_cu& bb) const;

    IF_CUDA_DEVICE_HOST
    inline void add_point(const Point_cu& p);

    /// Get x, y and z lenghts of the bouding box
    IF_CUDA_DEVICE_HOST
    inline Vec3_cu lengths() const;

    /// A valid bbox as lengths stricly superior to epsilon
    IF_CUDA_DEVICE_HOST
    inline bool is_valid( float eps = 0.00001f) const {
        return (pmax.x - pmin.x) > eps &&
               (pmax.y - pmin.y) > eps &&
               (pmax.z - pmin.z) > eps;
    }

    /// If the bbox represent the boundary of a grid with res.x * res .y * res.z
    /// Cells this compute the integer index 'pos' corresponds to.
    /// @param res : resolution (nb cells) of the grid in x, y and z directions
    /// @param pos : world position we want to find the corresponding frid index
    /// @return the grid index corresponding to the worlf position pos.
    /// @warning user must check for out of bounds indices.
    IF_CUDA_DEVICE_HOST
    inline Vec3i_cu index_grid_cell(Vec3i_cu res, Vec3_cu pos) const;

    /// Same as get_corner()
    inline void get_corners(std::vector<Point_cu>& corners);

    /// Get the ith corner position. Indexation is not the same as in
    /// 'get_bbox_corners()'
    /**
        @code

            6 +----+ 7
             /|   /|
          2 +----+3|
            |4+--|-+5
            |/   |/
            +----+
           0      1
        // Vertex 0 is pmin and vertex 7 pmax
        @endcode
    */
    IF_CUDA_DEVICE_HOST
    inline Point_cu get_corner(int i) const;


    Point_cu pmin;
    Point_cu pmax;
};
// =============================================================================

/** @struct OBBox_cu
  @brief Oriented bounding box
  To represent an oriented bouding box we use an easy to setup axis aligned
  bounding box (the 'bb' field) and associate a transformation 'tr'.
  one can find the position of the oriented bouding box by transforming the 'bb'
  points with 'tr'.

*/
struct OBBox_cu{
    OBBox_cu() : _tr( Transfo::identity() )
    {   }

    OBBox_cu(const BBox_cu& bb, const Transfo& tr):
        _bb(bb), _tr(tr)
    {   }

    /// @return the axis aligned bounding box which enclose the oriented bbox
    BBox_cu to_bbox() const {
        BBox_cu tmp;
        for(int i = 0; i < 8; ++i)
            tmp.add_point( _tr * _bb.get_corner(i) );
        return tmp;
    }

    BBox_cu _bb;
    Transfo _tr;
};
// =============================================================================

IF_CUDA_DEVICE_HOST inline
bool BBox_cu::inside(const Point_cu& p) const {
    return (p.x >= pmin.x) & (p.y >= pmin.y) & (p.z >= pmin.z) &
            (p.x <= pmax.x) & (p.y <= pmax.y) & (p.z <= pmax.z);
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
bool BBox_cu::isect(const Ray_cu& ray, float& tmin, float& tmax) const
{
    #ifndef __CUDA_ARCH__
    ///////////////
    // Host code //
    ///////////////

    //principle : compute the coordinates for each pair of slabs
    //(one per axis), and take the max of the three lower values,
    //and the min for the greater.
    float t0 = tmin;
    float t1 = tmax;

    for (int i = 0; i < 3; i++)
    {
        //compute the distance to travel along the ray
        //to reach each slab for the current axis.
        float inv_ray_dir = 1.f / ray._dir[i];

        //assume for the moment that the slab corresponding
        //to the min point is the nearest.
        float t_near = (pmin[i] - ray._pos[i]) * inv_ray_dir;
        float t_far  = (pmax[i] - ray._pos[i]) * inv_ray_dir;

        // rearrange if necessary.
        if(t_near > t_far)
        {
            float tmp = t_near;
            t_near = t_far;
            t_far  = tmp;
        }

        // update max and min.
        t0 = t_near > t0 ? t_near : t0;
        t1 = t_far  < t1 ? t_far  : t1;

        // no intersection detection.
        if(t0 > t1) return false;

    }

    //there is an intersection.
    assert( t0 >= tmin );
    assert( t1 <= tmax );

    tmin = t0;
    tmax = t1;

    return true;
    #else
    /////////////////
    // Device code //
    /////////////////
    float l1 = __fdividef(pmin.x - ray._pos.x, ray._dir.x);
    float l2 = __fdividef(pmax.x - ray._pos.x, ray._dir.x);
    tmin = fmaxf(fminf(l1,l2), tmin);
    tmax = fminf(fmaxf(l1,l2), tmax);

    l1 = __fdividef(pmin.y - ray._pos.y, ray._dir.y);
    l2 = __fdividef(pmax.y - ray._pos.y, ray._dir.y);
    tmin = fmaxf(fminf(l1,l2), tmin);
    tmax = fminf(fmaxf(l1,l2), tmax);

    l1 = __fdividef(pmin.z - ray._pos.z, ray._dir.z);
    l2 = __fdividef(pmax.z - ray._pos.z, ray._dir.z);
    tmin = fmaxf(fminf(l1,l2), tmin);
    tmax = fminf(fmaxf(l1,l2), tmax);

    return ((tmax >= tmin) & (tmax >= 0.f));
    #endif
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
BBox_cu BBox_cu::bbox_union(const BBox_cu& bb) const {
    return BBox_cu(fminf(pmin.x, bb.pmin.x), fminf(pmin.y, bb.pmin.y), fminf(pmin.z, bb.pmin.z),
                   fmaxf(pmax.x, bb.pmax.x), fmaxf(pmax.y, bb.pmax.y), fmaxf(pmax.z, bb.pmax.z));
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
BBox_cu BBox_cu::bbox_isect(const BBox_cu& bb) const {
    BBox_cu res =
            BBox_cu(fmaxf(pmin.x, bb.pmin.x), fmaxf(pmin.y, bb.pmin.y), fmaxf(pmin.z, bb.pmin.z),
                    fminf(pmax.x, bb.pmax.x), fminf(pmax.y, bb.pmax.y), fminf(pmax.z, bb.pmax.z));
    if((res.pmin.x > res.pmax.x) |
       (res.pmin.y > res.pmax.y) |
       (res.pmin.z > res.pmax.z)
      )
    {
        res = BBox_cu();
    }
    return res;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
void BBox_cu::add_point(const Point_cu& p) {
    pmin.x = fminf(p.x, pmin.x);
    pmin.y = fminf(p.y, pmin.y);
    pmin.z = fminf(p.z, pmin.z);
    pmax.x = fmaxf(p.x, pmax.x);
    pmax.y = fmaxf(p.y, pmax.y);
    pmax.z = fmaxf(p.z, pmax.z);
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST inline
Vec3_cu BBox_cu::lengths() const{ return pmax-pmin; }

// -----------------------------------------------------------------------------

inline void BBox_cu::get_corners(std::vector<Point_cu>& corners)
{
    corners.resize(8);
    for (int i = 0; i < 8; ++i)
        corners[i] = get_corner( i );
}

// -----------------------------------------------------------------------------

inline Point_cu BBox_cu::get_corner(int i) const
{
    Vec3_cu diff = pmax-pmin;
    diff.x *= (i      & 0x1);
    diff.y *= (i >> 1 & 0x1);
    diff.z *= (i >> 2 & 0x1);
    return Point_cu( pmin.x + diff.x, pmin.y + diff.y, pmin.z + diff.z);
}

// -----------------------------------------------------------------------------

Vec3i_cu BBox_cu::index_grid_cell(Vec3i_cu res, Vec3_cu p) const
{
    Vec3_cu cell_lengths = lengths().div( (Vec3_cu)res );

    // Local coords in the grid
    Vec3_cu lcl = p - pmin;
    Vec3_cu idx = lcl.div( cell_lengths );
    Vec3i_cu int_idx( (int)floorf(idx.x),
                      (int)floorf(idx.y),
                      (int)floorf(idx.z) );

    return int_idx;
}

// -----------------------------------------------------------------------------

#endif

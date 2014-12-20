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
#include "animesh_kers_colors.hpp"

#include "animesh.hpp"
#include "bbox2i_cu.hpp"

using namespace Cuda_utils;

// =============================================================================
namespace Animesh_colors {
// =============================================================================

/// Paint duplicated vertices. Often vertices are duplicated because they have
/// multiple texture coordinates or normals. This means these vertices are to be
/// with the same color
__device__ static
void paint(const Mesh::Packed_data& map, const float4& color, float4* colors)
{
    for(int i=0; i < map.nb_ocurrence; i++) {
        int idx = map.idx_data_unpacked + i;
        colors[idx] = color;
    }
}

// -----------------------------------------------------------------------------

static inline  __device__
float4 ssd_interpolation_colors( float fact )
{
    float r = 1.f;
    float g = 1.f - fact;
    float b = 0.f;

    if( isnan(fact)) return make_float4(1.f, 1.f, 1.f, 0.99f);
    else             return make_float4(  r,   g,   b, 0.99f);
}

// -----------------------------------------------------------------------------

template <class Attr_t>
struct Painter
{
    Painter(const Mesh::Packed_data* d_map,
            float4* d_colors,
            Attr_t* d_attr,
            Attr_t val) :
        _d_map(d_map),
        _d_colors(d_colors),
        _d_attr(d_attr),
        _val(val)
    {    }

    __device__ void update_attr(int idx, bool in){
        if( in ) _d_attr[idx] = _val;
        else     _val = _d_attr[idx];
    }

    const Mesh::Packed_data* _d_map;
    float4* _d_colors;
    Attr_t* _d_attr;
    Attr_t  _val;
};

// -----------------------------------------------------------------------------

struct Painter_ssd_lerp : public Painter<float>
{
    Painter_ssd_lerp(const Mesh::Packed_data* d_map,
                     float4* d_colors,
                     float* d_attr,
                     float val) :
        Painter<float>(d_map, d_colors, d_attr, val)
    { }

    __device__ void update_color(int idx) {
        paint(_d_map[idx], ssd_interpolation_colors(_val), _d_colors);
    }
};

// -----------------------------------------------------------------------------

struct Painter_cluster : public Painter<int>
{
    Painter_cluster(const Mesh::Packed_data* d_map,
                    float4* d_colors,
                    int* d_attr,
                    int val) :
        Painter<int>(d_map, d_colors, d_attr, val)
    { }

    __device__ void update_color(int idx) {
        const Color c = Color::pseudo_rand(_val);
        paint(_d_map[idx], make_float4(c.r, c.g, c.b, c.a), _d_colors);
    }
};

// -----------------------------------------------------------------------------

struct Depth_brush {

    /// Compute brush
    /// @param center : brush center
    /// (<b>origin is at the lower left of the screen</b>)
    static bool make(Depth_brush& brush, int rad, const Vec2i_cu center, int swidth, int sheight )
    {
        const int width = rad*2 - 1;
        const int x = center.x - rad - 1;
        const int y = center.y - rad - 1;
        BBox2i_cu box_screen(0, 0, swidth, sheight);
        BBox2i_cu box_brush(x, y, x+width, y+width);
        box_brush = box_brush.bbox_isect(box_screen);
        Vec2i_cu len = box_brush.lengths();
        if(len.x * len.y <= 0){
            brush._brush_width = 0;
            return false;
        }
        brush._brush_width = len.x;
        brush._box_brush   = box_brush;

        // Get depth buffer
        std::vector<float> depth( len.x*len.y );
        glReadPixels( box_brush.pmin.x, box_brush.pmin.y, len.x, len.y, GL_DEPTH_COMPONENT, GL_FLOAT, &(depth[0]) );
        brush._depth.malloc( depth.size() );
        brush._depth.copy_from( depth );
        return true;
    }

    bool empty() const { return _brush_width == 0; }

    /// @param wpt : projected point in screen space
    /// @return if inside the brush square
    __device__ bool is_inside(const Vec2i_cu& wpt){
        BBox2i_cu tmp = _box_brush;
        tmp.pmax -= 1;
        return tmp.inside(wpt);
    }

    /// @param wpt : projected point in screen space
    /// @param z : depth of the point
    /// @return if in front of the z-depth
    __device__ bool is_front(const Vec2i_cu& wpt, float z){
        // Projected point within the local box coordinates
        Vec2i_cu wpt_lcl = wpt;
        wpt_lcl.x -= _box_brush.pmin.x;
        wpt_lcl.y -= _box_brush.pmin.y;

        const int idx = wpt_lcl.y * _brush_width + wpt_lcl.x;

        return (idx >= 0 &&
                idx < _depth.size() &&
                (_depth[idx]+0.001f) >= z);
    }

    BBox2i_cu _box_brush;
    int _brush_width;
    DA_float _depth;
};

// -----------------------------------------------------------------------------

template <class Painter_t>
static __global__
void paint_kernel(Painter_t painter,
                  const Point_cu* in_verts,
                  int nb_verts,
                  Depth_brush brush,
                  const Transfo tr )
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_verts)
    {
        // Project onto screen plane
        const Point_cu pt = tr.project( in_verts[p] );
        // Compute integral coordinates of screen space
        const Vec2i_cu wpt((int)floorf(pt.x), (int)floorf(pt.y));
        // Check visibility according to brush bbox and depth
        bool in = brush.is_inside(wpt) && brush.is_front(wpt, pt.z);
        painter.update_attr(p, in);
        painter.update_color(p);
    }
}

// -----------------------------------------------------------------------------

void paint(EAnimesh::Paint_type mode,
           const Animesh::Paint_setup& setup,
           const Depth_brush& brush,
           const Transfo tr,
           const DA_Point_cu& in_verts,
           float* attr,
           const Mesh::Packed_data* map,
           float4* colors)
{
    const int block_size = 256;
    const int grid_size = (in_verts.size() + block_size - 1) / block_size;

    switch(mode)
    {
    case(EAnimesh::PT_SSD_INTERPOLATION):
    {
        Painter_ssd_lerp painter(map, colors, attr, setup._val);
        paint_kernel<<<grid_size, block_size>>>(painter, in_verts.ptr(), in_verts.size(), brush, tr);
    }break;

    default: break;
    }
}

}// ============================================================================

void Animesh::paint(EAnimesh::Paint_type mode,
                    const Paint_setup& setup,
                    const Camera& cam)
{
    using namespace Animesh_colors;

    const Transfo tr = cam.get_viewport_transfo() * cam.get_proj_transfo() * cam.get_eye_transfo();

    int rad = std::max(setup._brush_radius, 1);
    Vec2i_cu c(setup._x, cam.height()-setup._y);
    Depth_brush brush;
    if( !Depth_brush::make(brush, rad, c, cam.width(), cam.height()) )
        return;

    float4* d_colors = 0;
    _mesh->_color_bo.cuda_map_to(d_colors);

    const DA_Point_cu& in_verts = setup._rest_pose ? d_input_vertices : d_output_vertices;

    Animesh_colors::paint( mode,
                           setup,
                           brush,
                           tr,
                           in_verts,
                           d_ssd_interpolation_factor.ptr(),
                           d_packed_vert_map.ptr(),
                           d_colors);

    _mesh->_color_bo.cuda_unmap();
}

// -----------------------------------------------------------------------------

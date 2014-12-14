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

__global__
void cluster_colors_kernel(float4* d_colors,
                           const Mesh::Packed_data* d_map,
                           DA_int  d_vertices_nearest_bones)
{
    int n = d_vertices_nearest_bones.size();
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        int bone = d_vertices_nearest_bones[p];
        Color c = Color::pseudo_rand(bone);

        paint(d_map[p], make_float4(c.r, c.g, c.b, 0.99f), d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void ssd_interpolation_colors_kernel(float4* d_colors,
                                     const Mesh::Packed_data* d_map,
                                     DA_float d_ssd_interpolation_factor)
{
    int n = d_ssd_interpolation_factor.size();
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        // Poids ssd
        const float fact = d_ssd_interpolation_factor[p];
        paint(d_map[p], ssd_interpolation_colors(fact), d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void smoothing_colors_kernel(float4* d_colors,
                             const Mesh::Packed_data* d_map,
                             DA_float d_smoothing_factors)
{
    int n = d_smoothing_factors.size();
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        // Poids ssd
        const float f = d_smoothing_factors[p];
        float fact = f - 1.f;
        fact *= fact; // 2
        fact *= fact; // 4
        fact *= fact; // 16
        fact *= fact; // 32
        float r = 0.8f;
        float g = fact*0.8;
        float b = fact*0.8;

        if( isnan( f ) )
            paint(d_map[p], make_float4(1.f ,1.f , 1.f, 0.99f), d_colors);
        else if( f > 1.f )
            paint(d_map[p], make_float4(0.f , 1.f , 0.f, 0.99f), d_colors);
        else if( f < 0.f )
            paint(d_map[p], make_float4(0.f , 0.f , 1.f, 0.99f), d_colors);
        else
            paint(d_map[p], make_float4(r ,g, b, 0.99f), d_colors);
    }
}


// -----------------------------------------------------------------------------

__global__
void nearest_joint_colors_kernel(float4* d_colors,
                                 const Mesh::Packed_data* d_map,
                                 DA_int  d_vertices_nearest_joint)
{
    int n = d_vertices_nearest_joint.size();
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        int bone = d_vertices_nearest_joint[p];
        Color c = Color::pseudo_rand(bone);

        paint(d_map[p], make_float4(c.r, c.g, c.b, 0.99f), d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void normal_colors_kernel(float4* d_colors,
                          const Mesh::Packed_data* d_map,
                          const Vec3_cu* d_normal,
                          int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        // Poids ssd
        Vec3_cu n = d_normal[p];
        float r = n.x;
        float g = n.y;
        float b = n.z;

        if(isnan(r) || isnan(g) || isnan(b))
            r = g = b = 1.f;
        else if( sqrt(r*r + g*g + b*b) < 0.0001f )
            r = g = b = 0.f;
        else{
            n.normalize();
            r = n.x * 0.5f + 0.5f;
            g = n.y * 0.5f + 0.5f;
            b = n.z * 0.5f + 0.5f;
        }

        paint(d_map[p], make_float4(r ,g, b, 0.99f), d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void user_defined_colors_kernel(float4* d_colors,
                                const Mesh::Packed_data* d_map,
                                float4 color,
                                int n)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n)
        paint(d_map[p], color, d_colors);
}

// -----------------------------------------------------------------------------

__global__
void vert_state_colors_kernel(float4* d_colors,
                              const Mesh::Packed_data* d_map,
                              Device::Array<EAnimesh::Vert_state> d_vertices_state,
                              Device::Array<float4> d_vertices_states_color)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < d_vertices_state.size())
    {
        EAnimesh::Vert_state state = d_vertices_state[p];
        paint(d_map[p], d_vertices_states_color[state], d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void mvc_colors_kernels(float4* d_colors,
                        const Mesh::Packed_data* d_map,
                        const int* edge_list,
                        const int* edge_list_offsets,
                        const float* edge_mvc,
                        const int nb_verts)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < nb_verts)
    {
        const int offset = edge_list_offsets[2*p  ];
        const int nb_ngb = edge_list_offsets[2*p+1];
        float sum = 0.f;
        bool s = false;
        for(int i = offset; i < offset + nb_ngb; i++){
            //const int j = edge_list[i];
            const float mvc = edge_mvc[i];
            s = s || mvc < 0.f;
            sum += mvc;
        }

        float4 color;
        if( s )
            color = make_float4(0.f, 1.f, 1.f, 0.99f);
        else if( fabs(sum) < 0.00001f )
            color = make_float4(0.f, 1.f, 1.f, 0.99f);
        else if( sum  > 0.f )
            color = make_float4(1.f, 0.f, 0.f, 0.99f);
        else if( sum  < 0.f )
            color = make_float4(0.f, 0.f, 1.f, 0.99f);
        else if( isnan(sum) )
            color = make_float4(1.f, 1.f, 1.f, 0.99f);

        paint(d_map[p], color, d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void ssd_weights_colors_kernel(float4* d_colors,
                               const Mesh::Packed_data* d_map,
                               int joint_id,
                               Device::Array<int> d_jpv,
                               Device::Array<float> d_weights,
                               Device::Array<int> d_joints )
{
    int n = d_jpv.size()/2;
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        int start = d_jpv[p*2  ];
        int end   = start + d_jpv[p*2+1];

        int i = start;
        bool state = false;
        while(i < end){
            state = (d_joints[i] == joint_id) || state;
            if(state) break;
            i++;
        }

        Color c;
        if(!state){
            // no weight associated is blue
            c.set(0.f, 0.f, 1.f, 0.99f);
        }else{
            float w = d_weights[i];
            if(isnan(w)) c.set(1.f, 1.f,   1.f, 0.99f);
            else         c = Color::heat_color(w);
        }

        paint(d_map[p], make_float4(c.r, c.g, c.b, 0.99f), d_colors);
    }
}

// -----------------------------------------------------------------------------

__global__
void base_potential_colors_kernel(float4* d_colors,
                                  const Mesh::Packed_data* d_map,
                                  DA_float d_base_potential)
{
    int n = d_base_potential.size();
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < n){
        float pot = d_base_potential[p];

        Color c(0.f, 0.f, 0.f, 0.99f);

        if( isnan(pot) )
            c.r = c.g = c.b = 1.f;
        else if( pot < 0.f )
            c.r = c.g = c.b = 0.f;
        else{
            pot = 1.f - fmaxf(fminf((pot - 0.4f) * 5.f, 1.f), 0.f);
            c = Color::heat_color(pot);
        }

        paint(d_map[p], make_float4(c.r, c.g, c.b, 0.99f), d_colors);
    }
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

// -----------------------------------------------------------------------------
// Animesh methods implems related to color and painting
// -----------------------------------------------------------------------------

void Animesh::set_colors(EAnimesh::Color_type type,
                         float r,
                         float g,
                         float b,
                         float a)
{
    mesh_color = type;

    const int nb_vert = _mesh->get_nb_vertices();

    const int block_size = 256;
    const int grid_size = (d_input_vertices.size() + block_size - 1) / block_size;

    Mesh::Packed_data* d_map = d_packed_vert_map.ptr();
    float4* d_colors;
    _mesh->_color_bo.cuda_map_to(d_colors);

    switch(type)
    {
    case EAnimesh::BASE_POTENTIAL:
        Animesh_colors::base_potential_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, d_base_potential);
        break;
    case EAnimesh::GRAD_POTENTIAL:
    case EAnimesh::MVC_SUM:
        /////////////////////////////////////////////////////////////////////////// FIXME: TODO: use a proper enum
        /*
        Animesh_colors::gradient_potential_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, (Vec3_cu*)d_output_vertices.ptr(), d_output_vertices.size());
        */
        Animesh_colors::mvc_colors_kernels<<<grid_size, block_size>>>
            (d_colors, d_map, d_edge_list.ptr(), d_edge_list_offsets.ptr(), d_edge_mvc.ptr(), nb_vert);

        break;
    case EAnimesh::SSD_INTERPOLATION:
        Animesh_colors::ssd_interpolation_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, d_ssd_interpolation_factor);
        break;
    case EAnimesh::SMOOTHING_WEIGHTS:
        Animesh_colors::smoothing_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, d_input_smooth_factors);
        break;
    case EAnimesh::ANIM_SMOOTH_LAPLACIAN:
        Animesh_colors::smoothing_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, d_smooth_factors_laplacian);
        break;
    case EAnimesh::ANIM_SMOOTH_CONSERVATIVE:
        Animesh_colors::smoothing_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, d_smooth_factors_conservative);
        break;
    case EAnimesh::NEAREST_JOINT:
        Animesh_colors::nearest_joint_colors_kernel<<<grid_size, block_size>>>
            (d_colors,  d_map, d_vertices_nearest_joint);
        break;
    case EAnimesh::CLUSTER:
        Animesh_colors::cluster_colors_kernel<<<grid_size, block_size>>>
            (d_colors,  d_map, d_vertices_nearest_bones/*d_nearest_bone_in_device_mem*/);
        break;
    case EAnimesh::NORMAL:
    {
        Vec3_cu* normals = (Vec3_cu*)d_output_normals.ptr();
        Animesh_colors::normal_colors_kernel<<<grid_size, block_size>>>
            (d_colors, d_map, normals, _mesh->get_nb_vertices());
    }
        break;
    case EAnimesh::USER_DEFINED:
    {
        float4 color = make_float4(r, g, b, a);
        Animesh_colors::user_defined_colors_kernel
                <<<grid_size, block_size>>>
                (d_colors, d_map, color,  _mesh->get_nb_vertices());
    }
        break;
    case EAnimesh::VERTICES_STATE:
    {
        Animesh_colors::vert_state_colors_kernel<<<grid_size, block_size>>>
            (d_colors,  d_map, d_vertices_state, d_vertices_states_color);
    }
        break;
    case EAnimesh::SSD_WEIGHTS:
        assert(false); // use the method set_color_ssd_weight()
        break;
    }
    _mesh->_color_bo.cuda_unmap();
}

// -----------------------------------------------------------------------------

void Animesh::set_color_ssd_weight(int joint_id)
{
    mesh_color = EAnimesh::SSD_WEIGHTS;

    const int block_size = 256;
    const int grid_size = (d_input_vertices.size() + block_size - 1) / block_size;

    Mesh::Packed_data* d_map = d_packed_vert_map.ptr();
    float4* d_colors;
    _mesh->_color_bo.cuda_map_to(d_colors);

    Animesh_colors::ssd_weights_colors_kernel<<<grid_size, block_size>>>
        (d_colors, d_map, joint_id, d_jpv, d_weights, d_joints);
    _mesh->_color_bo.cuda_unmap();
}

// -----------------------------------------------------------------------------

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

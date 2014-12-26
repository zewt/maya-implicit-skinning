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
// =============================================================================
namespace Precomputed_env{
// =============================================================================

static void bind_local()
{
    d_offset.bind_tex(tex_offset_);

    d_anim_transform.bind_tex(tex_transform);

    d_grad_transform.bind_tex(tex_transform_grad);

    tex_grids.normalized = false;
    tex_grids.addressMode[0] = cudaAddressModeClamp;
    tex_grids.addressMode[1] = cudaAddressModeClamp;
    tex_grids.addressMode[2] = cudaAddressModeClamp;
    tex_grids.filterMode = cudaFilterModeLinear;
    d_block.bind_tex(tex_grids);
}

// -----------------------------------------------------------------------------

static void unbind_local()
{
    CUDA_SAFE_CALL( cudaUnbindTexture( tex_grids)          );
    CUDA_SAFE_CALL( cudaUnbindTexture( tex_transform)      );
    CUDA_SAFE_CALL( cudaUnbindTexture( tex_transform_grad) );
    CUDA_SAFE_CALL( cudaUnbindTexture( tex_offset_)        );
}

// -----------------------------------------------------------------------------

__device__
Transfo fetch_transform(int inst_id)
{
    struct{
        float4 a;
        float4 b;
        float4 c;
        float4 d;
    } s;

    s.a = tex1Dfetch(tex_transform, inst_id*4 + 0);
    s.b = tex1Dfetch(tex_transform, inst_id*4 + 1);
    s.c = tex1Dfetch(tex_transform, inst_id*4 + 2);
    s.d = tex1Dfetch(tex_transform, inst_id*4 + 3);

    return *reinterpret_cast<Transfo*>(&s);
}

// -----------------------------------------------------------------------------

__device__
Transfo fetch_transform_grad(int inst_id)
{
    struct{
        float4 a;
        float4 b;
        float4 c;
        float4 d;
    } s;

    s.a = tex1Dfetch(tex_transform_grad, inst_id*4 + 0);
    s.b = tex1Dfetch(tex_transform_grad, inst_id*4 + 1);
    s.c = tex1Dfetch(tex_transform_grad, inst_id*4 + 2);
    s.d = tex1Dfetch(tex_transform_grad, inst_id*4 + 3);

    return *reinterpret_cast<Transfo*>(&s);
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu fetch_offset(int inst_id){
    int4 off = tex1Dfetch(tex_offset_, inst_id);
    return Vec3_cu(off.x*GRID_RES, off.y*GRID_RES, off.z*GRID_RES);
}

// -----------------------------------------------------------------------------

__device__
bool is_in_grid(const Point_cu& pt, Vec3_cu off)
{
    const float res = (float)GRID_RES;
    off = off + 0.5f;

    //plus/minus one are hacks because I forgot to pad
    return ((pt.x >= off.x + 1.f        ) & (pt.y >= off.y + 1.f        ) & (pt.z >= off.z + 1.f       ) &
            (pt.x <  (off.x + res - 1.f)) & (pt.y <  (off.y + res - 1.f)) & (pt.z <  (off.z + res - 1.f)));
}

// -----------------------------------------------------------------------------

/// @param p    the 3d texture coordinate of tex_grid
/// @param grad the gradient at point p
/// @return the potential at point p
__device__
float fetch_grid(const Point_cu& p, Vec3_cu& grad)
{
    float4 res = tex3D(tex_grids, p.x, p.y, p.z);
    grad.x = res.y;
    grad.y = res.z;
    grad.z = res.w;
    return res.x;
}

// -----------------------------------------------------------------------------

__device__
float fetch_potential_and_gradient(int inst_id, const Point_cu& p, Vec3_cu& grad)
{
    Point_cu  r = fetch_transform(inst_id) * p;
    Vec3_cu off = fetch_offset(inst_id);

    r = r + off;
    if( is_in_grid( r, off ) )
    {
        float pot = fetch_grid(r, grad);
        grad = fetch_transform_grad(inst_id) * grad;
        return pot;
    }

    grad = Vec3_cu(0.f, 0.f, 0.f);
    return 0.f;
}

// -----------------------------------------------------------------------------

__device__
float fetch_potential(int inst_id, const Point_cu& p)
{
    Point_cu  r = fetch_transform(inst_id) * p;
    Vec3_cu off = fetch_offset(inst_id);

    r = r + off;

//    if( is_in_grid( r, off ) )
//    {
//        float4 res = tex3D(tex_grids, r.x, r.y, r.z);
//        return res.x;
//    }

    return 0.f;
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu fetch_gradient(int inst_id, const Point_cu& p)
{
    Point_cu  r = fetch_transform(inst_id) * p;
    Vec3_cu off = fetch_offset(inst_id);

    r = r + off;

//    if( is_in_grid( r, off ) )
//    {
//        float4 res = tex3D(tex_grids, r.x, r.y, r.z);
//        return Vec3_cu(res.y, res.z, res.w);
//    }

    return Vec3_cu(0.f, 0.f, 0.f);
}

}
// END PRECOMPUTED_ENV NAMESPACE ===============================================

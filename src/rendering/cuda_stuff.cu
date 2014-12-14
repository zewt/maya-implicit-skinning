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
#include "cuda_stuff.hpp"

// -----------------------------------------------------------------------------

#include <iostream>

// -----------------------------------------------------------------------------

#include "cuda_main_kernels.hpp"

// To limit dependancies to this header
// #include "cuda_ctrl.hpp"  we include this instead:

#include "potential_plane_ctrl.hpp"
#include "skeleton_ctrl.hpp"
#include "display_ctrl.hpp"
#include "color_ctrl.hpp"
// =============================================================================
namespace Cuda_ctrl{
// =============================================================================
extern Skeleton_ctrl        _skeleton;
extern Display_ctrl         _display;
extern Color_ctrl           _color;
extern Potential_plane_ctrl _potential_plane;
}
// =============================================================================

// -----------------------------------------------------------------------------

#include "timer.hpp"
#include "cuda_utils.hpp"
#include "filters.hpp"

#include "cuda_globals.hpp"
#include "globals.hpp"

#include "skeleton.hpp"
#include "cuda_globals.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif


#include "conversions.hpp"
#include "scene_enum.hpp"


// -----------------------------------------------------------------------------

void get_controller_values_from_tex(int inst_id, Cuda_utils::Host::Array<float2> vals)
{
    const int block_size = 16;
    const int grid_size  = (vals.size() + block_size - 1) / block_size;

    Cuda_utils::DA_float2 d_vals(vals.size());
    get_controller_values<<<block_size, grid_size>>>(d_vals, inst_id);

    vals.copy_from(d_vals);
}

// -----------------------------------------------------------------------------

void draw_controller(int inst_id, int x, int y, int w, int h)
{
    GLEnabledSave save_tex(GL_TEXTURE_2D, true, true);
    glDisable(GL_DEPTH_TEST);
    glBindTexture(GL_TEXTURE_2D, g_ctrl_frame_tex);
    GLViewportSave save_viewport;
    glViewport(x,y,w,h);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.f,1.f,0.f,1.f,0.f,1.f);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_QUADS);
    glTexCoord2f(0.f,1.f);
    glVertex2f(0.f,0.f);
    glTexCoord2f(1.f,1.f);
    glVertex2f(1.f,0.f);
    glTexCoord2f(1.f,0.f);
    glVertex2f(1.f,1.f);
    glTexCoord2f(0.f,0.f);
    glVertex2f(0.f,1.f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
    glColor4f(1.f,0.f,0.f,1.f);
    const float x0 = 0.121875f;
    const float x1 = 0.853125f;
    const float y0 = 0.175f;
    const float y1 = 0.666667f;
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.f);
    glBegin(GL_LINES);

    Cuda_utils::HA_float2 vals(101);
    get_controller_values_from_tex(inst_id, vals);
    for(int i = 0; i < 100; i++){
        float t0 = i     * 0.01f;
        float t1 = (i+1) * 0.01f;
        float f0 = vals[i  ].x;//Blending_env::eval_global_ctrl(cosf(t0 * M_PI));
        float f1 = vals[i+1].x;//Blending_env::eval_global_ctrl(cosf(t1 * M_PI));
        t0 = t0 * (x1 - x0) + x0; t1 = t1 * (x1 - x0) + x0;
        f0 = f0 * (y1 - y0) + y0; f1 = f1 * (y1 - y0) + y0;
        glVertex2f(t0, f0);
        glVertex2f(t1, f1);
    }
    glEnd();
    glDisable (GL_BLEND);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// =============================================================================
namespace Raytracing {
// =============================================================================


//DEBUG
#if 0
void kernel_debug(int3 steps, dim3 gridDim, dim3 blockDim)
{
    using namespace Cuda_ctrl;
    int w = _display._width;
    int h = _display._height;

    int acc = 0;

    dim3 blockIdx;
    dim3 threadIdx;
    for(blockIdx.x = 0; blockIdx.x < gridDim.x; blockIdx.x++)
    {
        for(blockIdx.y = 0; blockIdx.y < gridDim.y; blockIdx.y++)
        {
            for(threadIdx.x = 0; threadIdx.x < blockDim.x; threadIdx.x++)
            {
                for(threadIdx.y = 0; threadIdx.y < blockDim.y; threadIdx.y++)
                {
                    int px = blockIdx.x*blockDim.x + threadIdx.x;
                    int py = blockIdx.y*blockDim.y + threadIdx.y;

                    int size_block = (steps.x * steps.y);
                    int idx  = ((size_block/2) + steps.z) % size_block;
                    int offx = (idx % steps.x);
                    int offy = (idx / steps.x);
                    px = px * steps.x + offx;
                    py = py * steps.y + offy;

                    if (px >= w || py >= h)
                    {
                        // Outside
                    }
                    else
                    {
                        // inside
                        acc++;
                    }
                }
            }
        }
    }

    if(acc != w*h)
    {
        int a=0;
        a++;
    }

}
#endif
//DEBUG

/// Update buffers by raytracing the implicit primitive defined by the current
/// skeleton.
/// @param progressive : Activate the progressive raytracing Every call fills a
/// litlle bit more the buffers this is used for previsualisation. When the
/// scene parameters changes such as camera position resolution or the position
/// of the potential plane Buffers are erased and raytracing is done again
/// from the beginning
/// @return number of remaining calls to do to completely fill the buffers
int reset_buffers(const Camera& cam,
                   float4* d_rendu,
                   float* d_depth,
                   int width,
                   int height,
                   bool progressive)
{
    using namespace Cuda_ctrl;

    Context ctx;
    ctx.background      = _color.get(Color_ctrl::BACKGROUND);
    ctx.mat             = g_ray_material;
    ctx.potential_2d    = _display._potential_2d;
    ctx.draw_tree       = _display._raytrace_primitives;
    ctx.enable_lighting = _display._raytracing_lighting;
    ctx.cam             = Camera_data( cam );
    ctx.plane_n         = _potential_plane._normal;
    ctx.plane_org       = _potential_plane._org.to_point();
    ctx.step_len        = _display._ray_marching_step_length;
    ctx.nb_reflexion    = 0;
    ctx.enable_env_map  = _display._env_map;
    Potential_colors p_cl;
    p_cl.intern_color = _color.get(Color_ctrl::POTENTIAL_IN);
    p_cl.extern_color = _color.get(Color_ctrl::POTENTIAL_OUT);
    p_cl.negative_color = _color.get(Color_ctrl::POTENTIAL_NEG);
    p_cl.huge_color = _color.get(Color_ctrl::POTENTIAL_POS);
    ctx.potential_colors = p_cl;

    ctx.pbo = PBO_data(d_rendu, d_depth, 0, 0, width * MULTISAMPX, height * MULTISAMPY);

    int nb_samples = _display._nb_samples_res;
    static int passes = 0;

    nb_samples = nb_samples>width  ? width  : nb_samples;
    nb_samples = nb_samples>height ? height : nb_samples;

    int3 steps = {width/nb_samples, height/nb_samples, passes};

    dim3 dimBlock(BLOCK_SIZE_X,BLOCK_SIZE_Y);
    dim3 dimGrid((width  / steps.x * MULTISAMPX + dimBlock.x-1) / dimBlock.x,
                 (height / steps.y * MULTISAMPY + dimBlock.y-1) / dimBlock.y);

    // If any change in the scene occured we start raytracing again from the
    // begining
    if(has_changed(cam, ctx.plane_n, ctx.plane_org, width, height) ||
       _display._raytrace_again)
    {
        _display._raytrace_again = false;
        clean_buffers(d_rendu, d_depth, cam.get_far(), width, height);
        passes  = 0;
        steps.z = 0;
    }

    int nb_pass_max = (steps.x * steps.y);

    // if there are still empty pixels we raytrace them
    while( passes < nb_pass_max )
    {
        //kernel_debug(steps, dimGrid, dimBlock);
        ctx.grid  = dimGrid;
        ctx.block = dimBlock;
        ctx.steps = steps;

        Raytracing::trace( ctx );

        passes++;
        steps.z = passes;

        // When enabled we do the bloom effect at the last pass
        // FIXME: blooms needs the img_buff and bloom_buff from the current rendering context
//        if(_display._bloom && passes == nb_pass_max)
//            do_bloom(d_rendu, width*MULTISAMPX, height*MULTISAMPY);

        // progressive mode we do not raytrace all the pixels at the same time
        if(progressive) break;
    }

    return nb_pass_max - passes;
}

// -----------------------------------------------------------------------------

static void update_bbox()
{
    BBox_cu bbox;
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    if( g_skel == 0 ) return;
    int nb_bones = set.size() > 0 ? set.size() : g_skel->nb_joints();
    for(int i = 0; i < nb_bones; i++)
    {
        int idx = set.size() > 0 ? set[i] : i;

        const Bone* b = g_skel->get_bone( idx );
        bbox = bbox.bbox_union( b->get_obbox().to_bbox() );

    }

    Raytracing::update_bbox( bbox );
}

// -----------------------------------------------------------------------------


/// raytrace the current implicit model
/// @param d_rendu Intermediate buffer used to raytrace
/// @param progressive Activate progressive raytracing : only some pixels are
/// computed at each call. The result is blurred
/// @return if the raytracing is complete (happens when progressive mode is
/// activated)
bool raytrace_implicit(const Camera& cam,
                       float4* d_rendu,
                       float* d_rendu_depth,
                       int* d_img_buf,
                       unsigned* d_depth_buf,
                       int width,
                       int height,
                       bool progressive)
{
    using namespace Cuda_ctrl;

    int passes;
    const std::vector<int>& joint_set = _skeleton.get_selection_set();

    // Update the bounding box of the union of implicit surfaces to accelerate
    // raytracing a little
    update_bbox();
    Raytracing::set_partial_tree_eval( joint_set.size() == 0);
    if (joint_set.size()) {
        Raytracing::set_bone_to_trace( joint_set );
    }

    passes = reset_buffers(cam, d_rendu, d_rendu_depth, width, height, progressive);


    dim3 dimBlock_(BLOCK_SIZE_X,BLOCK_SIZE_Y);
    dim3 dimGrid_((width * MULTISAMPX + dimBlock_.x-1) / dimBlock_.x,
                  (height* MULTISAMPY + dimBlock_.y-1) / dimBlock_.y);


    bool do_filter  = passes > 0; // Finish we do not filter
    int block_x   = width  / _display._nb_samples_res;
    int block_y   = height / _display._nb_samples_res;
    // The filter size is proportionnal to the square root of
    // number of passes remaining
    float percent = sqrt((float)max(passes, 1) / (float)(block_y*block_x));
    // Filter size is minimum 16 because higher sizes are really too slow
    int filter_size = min(16, 1+max( (int)(block_x*percent), (int)(block_y*percent)));

    flatten_image<<<dimGrid_, dimBlock_ >>>(d_rendu,
                                            d_rendu_depth,
                                            d_img_buf,
                                            d_depth_buf,
                                            width*MULTISAMPX,
                                            height*MULTISAMPY,
                                            do_filter,
                                            filter_size,
                                            false);

    return passes == 0;
}

} // END RAYTRACING ============================================================


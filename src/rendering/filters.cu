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
#include "filters.hpp"

#include "packed_data_struct.hpp"
#include "macros.hpp"
#include "cuda_globals.hpp"

// Some parameters to tweak the bloom effect
#define BASE_BLOOM (0.017f*0.5f)
#define BLOOM_MUL (5.5f)
#define BLOOM_INCR (0.0f)

// -----------------------------------------------------------------------------

#define znew (z=36969*(z&65535)+(z>>16))
#define wnew (w=18000*(w&65535)+(w>>16))
#define MWC ((z<<16)+w)

__device__
int pack (const float4& s)
{
    int nr = ((int)(fminf(s.x*255.f,255.f)));
    int ng = ((int)(fminf(s.y*255.f,255.f)))<<8;
    int nb = ((int)(fminf(s.z*255.f,255.f)))<<16;
    return nr |ng |nb;
}

// -----------------------------------------------------------------------------

__device__
int pack_dither (const float4& s, int px, int py)
{
    int z = px, w = py;
    z = 1103515245 * z + 12345;
    w = 1103515245 * w + 12345;
    znew;	wnew;
    float x0 = MWC * 1.f / (0xFFFFFFFF);
    znew;	wnew;
    float x1 = MWC * 1.f / (0xFFFFFFFF);
    znew;	wnew;
    float x2 = MWC * 1.f / (0xFFFFFFFF);
    float fr = fminf(s.x * 255.f,255.f);
    float fg = fminf(s.y * 255.f,255.f);
    float fb = fminf(s.z * 255.f,255.f);
    int ir = fr;
    int ig = fg;
    int ib = fb;
    int nr = (((fr - ir) > x0)?ir+1:ir);
    int ng = ((((fg - ig) > x1)?ig+1:ig)<<8);
    int nb = ((((fb - ib) > x2)?ib+1:ib)<<16);
    return nr |ng |nb;
}

// -----------------------------------------------------------------------------

/// Additive filter.
/// Given an image stored linearly in 'img' of size (width, height)
/// We add every pixels of the square window with length 'filter_size'
/// centered at (px, py).
/// @note negative values in 'img' are ignored.
/// @param out_depth : the mean value of the depth inside the window.
/// @return the mean value of each color channel inside the window
__device__
float4 additive_filter(const float4* img,
                       const float* depth_buf,
                       int px, int py,
                       int width, int height,
                       int filter_size,
                       float& out_depth)
{
    // TODO: we can accelerate the filtering by loading into shared memory the
    // pixel block corresponding to the thread block.

    // Load px py in shared memory
    // synchronize threads of the block

    // Do the filtering using the shared memory when possible
    px -= (filter_size/2);
    py -= (filter_size/2);
    float4 new_val = {0.f, 0.f, 0.f, 0.f};
    float new_depth_val = 0.f;
    float contrib = 0.f;
    for(int i=0; i < filter_size; i++)
    {
        for(int j=0; j < filter_size; j++)
        {
            int idx = (px+i) + (py+j) * width;
            if(((px+i) < width) & ((px+i) >= 0) & ((py+j) < height) & ((py+j) >=0))
            {
                if(img[idx].x >=0.f)
                {
                    new_val.x += img[idx].x;
                    new_val.y += img[idx].y;
                    new_val.z += img[idx].z;
                    new_val.w += img[idx].w;
                    new_depth_val += depth_buf[idx];
                    contrib += 1.f;
                }
            }
        }
    }

    out_depth = new_depth_val/contrib, 0.f;
    contrib  *= (MULTISAMPX*MULTISAMPY);
    new_val.x = fmaxf(new_val.x/contrib, 0.f);
    new_val.y = fmaxf(new_val.y/contrib, 0.f);
    new_val.z = fmaxf(new_val.z/contrib, 0.f);
    new_val.w = fmaxf(new_val.w/contrib, 0.f);

    return new_val;
}


// -----------------------------------------------------------------------------

__global__
void flatten_image(const float4* in_color,
                   const float*  in_depth,
                   int*      out_rgb24,
                   unsigned* out_depth,
                   int width,
                   int height,
                   bool do_filter,
                   int filter_size,
                   bool dither)
{
    const int px = blockIdx.x * BLOCK_SIZE_X + threadIdx.x;
    const int py = blockIdx.y * BLOCK_SIZE_Y + threadIdx.y;
    const int idx = py * width + px;
    if(idx < width * height)
    {
        float depth;
        float4 pix_val;

        if(do_filter)
            pix_val = additive_filter(in_color, in_depth, px, py, width, height, filter_size, depth);
        else
        {
            pix_val = in_color[idx];
            depth   = in_depth[idx];
        }

        out_rgb24[idx] = dither ? pack_dither(pix_val, px, py) : pack(pix_val);
        out_depth[idx] = *reinterpret_cast<unsigned*>(&depth);
    }
}

// -----------------------------------------------------------------------------

__global__
void clean_pbo(int* color, unsigned* depth, int n, float4 val)
{
    const int p = threadIdx.x + blockDim.x * blockDim.y * blockIdx.x;
    if(p < n){
        color[p] = int(val.x *255) + (int(val.y*255)<<8) + (int(val.z*255)<<16);
        float depth_value = 1.f;
        depth[p] = *reinterpret_cast<unsigned*>(&depth_value);
    }
}

// -----------------------------------------------------------------------------

void clean_pbos(int* color,
                unsigned* depth,
                int width,
                int height,
                float4 cl_color)
{
    int nb_pixels = width * MULTISAMPX * height * MULTISAMPY;
    int dimblock = BLOCK_SIZE_X * BLOCK_SIZE_Y;
    int dimgrid = (nb_pixels + dimblock -1) / dimblock;

    clean_pbo<<<dimgrid, dimblock >>>(color, depth, nb_pixels, cl_color);

}

// -----------------------------------------------------------------------------

__global__
void clean_buff(float4* buff, float* depth, int n, float4 val, float depth_val)
{
    const int p = threadIdx.x + blockDim.x * blockDim.y * blockIdx.x;
    if(p < n){
        buff[p]  = make_float4(val.x, val.y, val.z, val.w);
        depth[p] = depth_val;
    }
}

// -----------------------------------------------------------------------------

/// Fill 'd_buff' with zeros
void clean_buffers(float4* d_buff_, float* d_depth_, float far_, int width_, int height_)
{
    int nb_pixels = width_ * MULTISAMPX * height_ * MULTISAMPY;

    int dimblock = BLOCK_SIZE_X * BLOCK_SIZE_Y;
    int dimgrid = (nb_pixels + dimblock -1) / dimblock;

    float4 clear_val = make_float4(-1.f, -1.f, -1.f, -1.f);
    clean_buff<<<dimgrid, dimblock >>>(d_buff_, d_depth_, nb_pixels, clear_val, far_);
}

// -----------------------------------------------------------------------------

// =============================================================================
namespace Bloom {
// =============================================================================

__device__
float4 operator+(float4 a, float4 b){
    return make_float4(a.x+b.x, a.y+b.y,a.z+b.z, a.w +b.w);
}

__device__
float4 operator*(float4 a, float f){
    return make_float4(a.x*f, a.y*f, a.z*f, a.w*f);
}

__device__
float4 operator*(float4 a, float4 b){
    return make_float4(a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w);
}

__device__ float4
lerp(const float4& a, const float4& b, float f){
    return a * f  + b * (1.f - f);
}

// -----------------------------------------------------------------------------

__global__
void copy_pbo_to_img(PBO_data pbo, float4* img)
{
    const int px = pbo.start_x + blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = pbo.start_y + blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    const int p = py * pbo.width + px;
    img[p] = pbo.d_rendu[p];

}

// -----------------------------------------------------------------------------

__global__
void copy_img_to_pbo(PBO_data pbo, float4* img)
{
    const int px = pbo.start_x + blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = pbo.start_y + blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    const int p = py * pbo.width + px;
    pbo.d_rendu[p] = img[p];

}

// -----------------------------------------------------------------------------

__global__
void clean_img(PBO_data pbo, float4* img)
{
    const int px = pbo.start_x + blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = pbo.start_y + blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    const int p = py * pbo.width + px;
    img[2*p] = make_float4(0.f, 0.f, 0.f, 0.f);
    img[2*p+1] = make_float4(0.f, 0.f, 0.f, 0.f);
}

// -----------------------------------------------------------------------------

__global__
void pre_compute_bloom(PBO_data pbo, float4* img, float4* bloom, int lvl)
{
    const int px = pbo.start_x + blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = pbo.start_y + blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    int w = pbo.width;
    int h = pbo.height;
    int offset = 0;
    for(int i = 0; i < lvl; i++){
        offset += w*h;
        w /= 2;
        h /= 2;
    }
    float4* img_src = &(img[offset]);
    float4* img_dst = &(img[offset+w*h]);
    w /= 2;
    h /= 2;
    if(px < w & py < h){
        float4 a = img_src[2*px +  4 * w * py];
        float4 b = img_src[2*px + 1 +  4 * w * py];
        float4 c = img_src[2*px +  (2 * py + 1 )* 2*w];
        float4 d = img_src[2*px + 1 +  (2 * py + 1)*2*w];
        img_dst[px + w*py] = (a+b+c+d)*0.25f;
    }
}

// -----------------------------------------------------------------------------

__global__
void compute_bloom0(PBO_data pbo, float4* img, float4* bloom, int lvls)
{
    const int px = pbo.start_x + blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = pbo.start_y + blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    int w = pbo.width;
    int h = pbo.height;
    float4* img_src = img;
    float4* bm_dst = bloom;
    for(int i = 0; i < lvls; i++){
        if(px < w & py < h){
            float4 e =  img_src[px + w*py];
            float4 a = 	(px>0)?img_src[px-1 + w*py]:e;
            float4 b = 	(px<w-1)?img_src[px+1 + w*py]:e;
            float4 c = 	(py>0)?img_src[px + w*(py-1)]:e;
            float4 d = 	(py<w-1)?img_src[px + w*(py+1)]:e;
            float4 f = 	(px>0 & py>0)?img_src[px-1 + w*(py-1)]:e;
            float4 g = 	(px>0 & py<w-1)?img_src[px-1 + w*(py+1)]:e;
            float4 h = 	(px<w-1 & py>0)?img_src[px+1 + w*(py-1)]:e;
            float4 k = 	(px<w-1 & py<w-1)?img_src[px+1 + w*(py+1)]:e;
            float4 res = (f + g + h + k)*(1.f/16.f)+ (a + b + c + d)*(1.f/8.f) + e*(1.f/4.f);
            bm_dst[px + w*py] = res;
        }
        bm_dst = &bm_dst[w*h];
        img_src = &img_src[w*h];
        w /= 2;
        h /= 2;
    }
}

// -----------------------------------------------------------------------------

__global__
void compute_bloom(PBO_data pbo, float4* img, float4* bloom, int lvls)
{
    const int px = pbo.start_x + blockIdx.x*BLOCK_SIZE_X + threadIdx.x;
    const int py = pbo.start_y + blockIdx.y*BLOCK_SIZE_Y + threadIdx.y;
    int w = pbo.width;
    int h = pbo.height;

    float pqx = px;
    float pqy = py;
    float factor = BASE_BLOOM;
    float4 res = make_float4(0.f,0.f,0.f,0.f);
    float4* bm_src = bloom;
    //res = bm_src[qx/2 + (w/2) * (qy/2) + w*h];
    //res = bm_src[qx + w*qy];
    int offset = 0;
    for(int i = 0; i < lvls; i++){
        factor *= BLOOM_MUL;
        factor += BLOOM_INCR;
        int qx = pqx;
        int qy = pqy;
        float dx = pqx-qx;
        float dy = pqy-qy;
        float4 color_a = bm_src[qx + w*qy + offset];
        float4 color_b = (qx<w-1)?bm_src[qx+1 + w*qy + offset]:color_a;
        float4 color_c = (qy<h-1)?bm_src[qx + w*(qy+1) + offset]:color_a;
        float4 color_d = (qx<w-1)?((qy<h-1)?bm_src[qx+1 + w*(qy+1) + offset]:color_b):
            ((qy<h-1)?color_c:color_a);

        float4 color_lvl = lerp(lerp(color_d, color_c, dx),lerp(color_b, color_a, dx),dy);
        color_lvl.x *= color_lvl.x;
        color_lvl.y *= color_lvl.y;
        color_lvl.z *= color_lvl.z;
        color_lvl.w *= color_lvl.w;
        res = res+ color_lvl * factor;

        offset += w*h;
        pqx = (pqx-0.5f)*0.5f;
        pqy = (pqy-0.5f)*0.5f;
        w /= 2;
        h /= 2;

    }

    /*int result;
    w = pbo.width;
    h = pbo.height;
    int qx = px;
    int qy = py;
    offset = 0;
    for(int i = 0; i < 6; i++){
        offset += w*h;
        w /= 2;
        h /= 2;
        qx /= 2;
        qy /= 2;
    }
    */

    const int p = py * pbo.width + px;
    img[p] = img[p] + res;//
    //img[p] = img[qx + w*qy + offset];

}

}// END BLOOM NAMESPACE =========================================================

void do_bloom(float4* d_rendu,
              float4* d_img_buff,
              float4* d_bloom_buff,
              int width, int height)
{
    using namespace Bloom;
    PBO_data pbo_d(d_rendu, NULL, 0, 0, width, height);

    dim3 dimBlock(BLOCK_SIZE_X,BLOCK_SIZE_Y);
    dim3 dimGrid(width/BLOCK_SIZE_X, height/BLOCK_SIZE_Y);
    dim3 dimGrid0((width + 2*BLOCK_SIZE_X-1)/(2*BLOCK_SIZE_X),
                  (height + 2*BLOCK_SIZE_Y-1)/(2*BLOCK_SIZE_Y));
    dim3 dimGrid1((width + 4*BLOCK_SIZE_X-1)/(4*BLOCK_SIZE_X),
                  (height + 4*BLOCK_SIZE_Y-1)/(4*BLOCK_SIZE_Y));
    dim3 dimGrid2((width + 8*BLOCK_SIZE_X-1)/(8*BLOCK_SIZE_X),
                  (height + 8*BLOCK_SIZE_Y-1)/(8*BLOCK_SIZE_Y));
    dim3 dimGrid3((width + 16*BLOCK_SIZE_X-1)/(16*BLOCK_SIZE_X),
                  (height + 16*BLOCK_SIZE_Y-1)/(16*BLOCK_SIZE_Y));
    dim3 dimGrid4((width + 32*BLOCK_SIZE_X-1)/(32*BLOCK_SIZE_X),
                  (height + 32*BLOCK_SIZE_Y-1)/(32*BLOCK_SIZE_Y));
    dim3 dimGrid5((width + 64*BLOCK_SIZE_X-1)/(64*BLOCK_SIZE_X),
                  (height + 64*BLOCK_SIZE_Y-1)/(64*BLOCK_SIZE_Y));
    dim3 dimGrid6((width + 128*BLOCK_SIZE_X-1)/(128*BLOCK_SIZE_X),
                  (height + 128*BLOCK_SIZE_Y-1)/(128*BLOCK_SIZE_Y));
    dim3 dimGrid7((width + 256*BLOCK_SIZE_X-1)/(256*BLOCK_SIZE_X),
                  (height + 256*BLOCK_SIZE_Y-1)/(256*BLOCK_SIZE_Y));
    clean_img<<<dimGrid, dimBlock >>>(pbo_d, d_img_buff);
    copy_pbo_to_img<<<dimGrid, dimBlock >>>(pbo_d, d_img_buff);
    int k = 2;
    pre_compute_bloom<<<dimGrid0, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 0);
    pre_compute_bloom<<<dimGrid1, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 1);

    if(width % 8 == 0 && height % 8 == 0){
        pre_compute_bloom<<<dimGrid2, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 2);
        k++;
    }
    if(width % 16 == 0 && height % 16 == 0){
        pre_compute_bloom<<<dimGrid3, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 3);
        k++;
    }
    if(width % 32 == 0 && height % 32 == 0){
        pre_compute_bloom<<<dimGrid4, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 4);
        k++;
    }
    if(width % 64 == 0 && height % 64 == 0){
        pre_compute_bloom<<<dimGrid5, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 5);
        k++;
    }
    if(width % 128 == 0 && height % 128 == 0){
        pre_compute_bloom<<<dimGrid4, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 6);
        k++;
    }
    if(width % 256 == 0 && height % 256 == 0){
        pre_compute_bloom<<<dimGrid5, dimBlock >>>(pbo_d, d_img_buff,d_bloom_buff, 7);
        k++;
    }
    compute_bloom0<<<dimGrid, dimBlock >>>(pbo_d, d_img_buff, d_bloom_buff, k);
    compute_bloom<<<dimGrid, dimBlock >>>(pbo_d, d_img_buff, d_bloom_buff, k);
    copy_img_to_pbo<<<dimGrid, dimBlock >>>(pbo_d, d_img_buff);
}

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
#include "textures_env.hpp"

#include "cuda_utils.hpp"
#include "ppm_loader.hpp"
#include "vec3_cu.hpp"

#include "macros.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// =============================================================================
namespace Textures_env{
// =============================================================================

extern const int light_envmap_downscale = 100;

cudaArray* array_envmap;
cudaArray* array_light_envmap;
cudaArray* blob_tex;
cudaArray* array_extrusion_mask[2];
cudaArray* array_extrusion_gradient[2];

int envmapx, envmapy;
int blobx, bloby;
int extrusionx[2];
int extrusiony[2];
int* img_envmap = 0;
int* img_light_envmap = 0;
int* blob_img = 0;
float* img_extrusion_mask[2];
float2* img_extrusion_gradient[2];

bool img_ok = false;
bool blob_ok = false;
bool extrusion_ok = false;
bool bind_ok = false;

static void compute_light_envmap();

// -----------------------------------------------------------------------------

void load_envmap(char* filename)
{
    if( Ppm_loader::read(filename, envmapx, envmapy, img_envmap) ){
        img_ok = true;
    }
    compute_light_envmap();
    /*img_envmap = img_light_envmap;
        envmapx /= light_envmap_downscale;
        envmapy /= light_envmap_downscale;*/
}

// -----------------------------------------------------------------------------

void load_blob_tex(char* filename)
{
    if( Ppm_loader::read(filename, blobx, bloby, blob_img)){
        blob_ok = true;
    }
}

// -----------------------------------------------------------------------------

void load_extrusion_tex(char* filename, int n)
{
    int* img;
    if( Ppm_loader::read(filename, extrusionx[n], extrusiony[n], img) )
    {
        img_extrusion_mask[n] = new float[extrusionx[n]* extrusiony[n]];
        img_extrusion_gradient[n] = new float2[extrusionx[n]* extrusiony[n]];
        for(int p = 0 ; p < extrusionx[n] * extrusiony[n]; p++){
            img_extrusion_mask[n][p] = (img[p] & 0x000000FF) * (1.f / 255.f);
        }
        float f_res_x = (float)extrusionx[n];
        float f_res_y = (float)extrusiony[n];
        for(int j = 0; j < extrusiony[n]; j++)
        {
            for(int i = 0 ; i < extrusionx[n]; i++)
            {
                int p = i + j * extrusionx[n];
                /*float dx = ((i < extrusionx-1)?img_extrusion_mask[p+1]:img_extrusion_mask[p])
      -((i > 0)?img_extrusion_mask[p-1]:img_extrusion_mask[p]);
     float dy = ((j < extrusiony-1)?img_extrusion_mask[p+extrusionx]:img_extrusion_mask[p])
     -((j > 0)?img_extrusion_mask[p-extrusionx]:img_extrusion_mask[p]);*/
                int RDS = 10;
                int i0 = (i >= RDS)?i-RDS:0;
                int i1 = (i + RDS < extrusionx[n])?i+RDS:(extrusionx[n]-1);
                int j0 = (j >= RDS)?j-RDS:0;
                int j1 = (j + RDS < extrusiony[n])?j+RDS:(extrusiony[n]-1);
                float dx = img_extrusion_mask[n][i1 + j*extrusionx[n]]
                        - img_extrusion_mask[n][i0 + j*extrusionx[n]];
                float dy = img_extrusion_mask[n][i + j1*extrusionx[n]]
                        - img_extrusion_mask[n][i + j0*extrusionx[n]];
                float lx = f_res_x / (i1 - i0);
                float ly = f_res_y / (j1 - j0);
                img_extrusion_gradient[n][p] = make_float2(dx*lx,dy*ly);
            }
        }
        extrusion_ok = true;
        free(img);
    }
}

// -----------------------------------------------------------------------------

void clean_env()
{
    if(bind_ok){
        CUDA_SAFE_CALL(cudaFreeArray(array_envmap));
        if(blob_ok){
            CUDA_SAFE_CALL(cudaFreeArray(blob_tex));
        }
        if(extrusion_ok){
            CUDA_SAFE_CALL(cudaFreeArray(array_extrusion_mask[0]));
            CUDA_SAFE_CALL(cudaFreeArray(array_extrusion_mask[1]));
        }
    }

    free(img_envmap);
    free(blob_img);

    delete[] img_light_envmap;
    delete[] img_extrusion_mask[0];
    delete[] img_extrusion_mask[1];
    delete[] img_extrusion_gradient[0];
    delete[] img_extrusion_gradient[1];
}

// -----------------------------------------------------------------------------

static Vec3_cu generate_vector(float theta, float phi)
{
    float cosphi = cosf(phi);
    float sinphi = sinf(phi);
    float costheta = cosf(theta);
    float sintheta = sinf(theta);
    float y = sintheta;
    float x = costheta * cosphi;
    float z = costheta * sinphi;
    return Vec3_cu(x, y, z);
}

// -----------------------------------------------------------------------------

static void compute_spherical_coordinates(const Vec3_cu& d, float& theta, float& phi)
{
    Vec3_cu u(d.x, 0.f, d.z);
    u = u.normalized();
    float cosphi = u.x;
    float sinphi = u.z;
    float costheta = d.dot(u);
    float sintheta = -d.y;
    phi = atan2f(sinphi, cosphi)/(2*M_PI) + 0.5f;
    phi = fminf(fmaxf(0.f,phi),1.f);
    theta = atan2f(sintheta,costheta)/M_PI + 0.5f;
    theta = fminf(fmaxf(0.f,theta),1.f);
}

// -----------------------------------------------------------------------------

static void compute_light_envmap()
{
    int size_x = envmapx / light_envmap_downscale;
    int size_y = envmapy / light_envmap_downscale;
    int downscale = 10;
    int size_xx = envmapx / downscale;
    int size_yy = envmapy / downscale;
    img_light_envmap = new int[size_x * size_y];
    float3* reduced = new float3[size_xx * size_yy];
    for(int i = 0; i < size_xx; i++){
        int offset_x = i*downscale;
        for(int j = 0; j < size_yy; j++){
            float r = 0.f, g = 0.f, b = 0.f;
            int offset_y = j*downscale;
            for(int x = 0; x < downscale; x++){
                for(int y = 0; y < downscale; y++){
                    int current = img_envmap[(offset_x + x)+(offset_y + y)*envmapx];
                    float dr = ((current >> 16) & 0x000000FF) * 1.f / 255;
                    float dg = ((current >> 8) & 0x000000FF) * 1.f / 255;
                    float db = (current & 0x000000FF) * 1.f / 255;
                    r += dr; g += dg; b += db;
                }
            }
            float factor = (float)(downscale*downscale);
            r /= factor;
            g /= factor;
            b /= factor;
            reduced[i + size_xx * j] = make_float3(r,g,b);
        }
    }


    for(int i = 0; i < size_x-1; i++){
        float phi = i * 2.f * M_PI / (size_x) ;
        for(int j = 0; j < size_y; j++){
            float theta = j * M_PI / (size_y) - M_PI * 0.5f;
            Vec3_cu h = generate_vector(theta,phi);
            float r = 0.f, g = 0.f, b = 0.f;
            for(int k = 0; k < 10000; k++){
                Vec3_cu v = Vec3_cu::random(1.f);
                v = v.normalized();
                float dot = h.dot(v);
                if(dot < 0.f){
                    v = -v;
                    dot = -dot;
                }

                float xf, yf;
                compute_spherical_coordinates(v, yf, xf);
                xf = (xf >= 1.f)? 0.f: xf;
                yf = (yf >= 1.f)? 0.f: yf;
                int x = (int)xf * size_xx;
                int y = (int)yf * size_yy;
                float3 value = reduced[x + size_xx * y];
                const float floor = 0.99f;
                if(value.x > floor & value.y > floor & value.z > floor){
                    value.x *= 20.f;
                    value.y *= 20.f;
                    value.z *= 20.f;
                }
                r += value.x*dot; g += value.y*dot; b += value.z*dot;
            }
            //float3 val = reduced[i + size_x * j];
            //r = val.x; g = val.y; b = val.z;
            int r0 = (int)(r * 0.0255f * M_PI*0.8f);
            int g0 = (int)(g * 0.0255f * M_PI*0.8f);
            int b0 = (int)(b * 0.0255f * M_PI*0.8f);
            r0 = min(r0,255);
            g0 = min(g0,255);
            b0 = min(b0,255);
            img_light_envmap[i + size_x*j] = (r0 << 16) | (g0 << 8) | b0;
        }
    }


    for(int j = 0; j < size_y; j++){
        img_light_envmap[size_x*j+size_x-1] = img_light_envmap[size_x*j];
    }

    for(int i = 0; i < size_x; i++){
        img_light_envmap[i] = img_light_envmap[0];
        img_light_envmap[i+size_x*(size_y-1)] = img_light_envmap[size_x*(size_y-1)];
    }

    delete[] reduced;

}
// -----------------------------------------------------------------------------

}// END ImgTextures NAMESPACE ==================================================


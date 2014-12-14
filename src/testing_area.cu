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
#include "intersection.hpp"

//#include "cuda_utils.hpp"
#include "cuda_compiler_interop.hpp"

#include <cassert>
#include <stdio.h>


__device__
void test(){
    FORBID_HOST_CALL();
}


#if 0
#include "cuda_current_device.hpp"

#include <vector>


/**
 *  @file testing_area.cu
 *  @brief Just a sand box to do whatever tests you need to
 */

__global__
void tri_inter(Inter::Triangle tr0, Inter::Triangle tr1,
               Inter::Point* p0,
               Inter::Point* p1,
               bool* r)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if(p < 1){
        bool coplanar;
        *r = Inter::tri_tri(tr0, tr1, *p0, *p1, coplanar);
    }
}




void tri_inter__(Inter::Triangle tr0, Inter::Triangle tr1,
                 Inter::Point* p0,
                 Inter::Point* p1,
                 bool* r)
{
    CUDA_LAUNCH_ARRAY(tri_inter, 8, 10, tr0, tr1, p0, p1, r);

}

#include <QTime>
#include "g_scene_tree.hpp"
#include "glsave.hpp"
#include "globals.hpp"
#include "cuda_ctrl.hpp"

QTime t;

void test_inter(Vec3_cu p)
{
#if 0
    /// Find two meshes
    Scene_tree::iterator it = g_scene_tree->begin();
    Obj_mesh* meshes[2];
    int acc = 0;
    for (; it != g_scene_tree->end(); ++it) {
        Obj* o = *it;
        if(o->type_object() == EObj::MESH)
        {
            meshes[acc] = (Obj_mesh*)o;
            acc++;
        }
        if(acc == 2) break;
    }

    if(acc != 2) return;


    meshes[1] = meshes[0];////////////////////////////////////////////////////////////
    const Mesh& m0 = meshes[0]->get_mesh();
    const Mesh& m1 = meshes[1]->get_mesh();

    const Transfo tr0 = meshes[0]->frame();
    const Transfo tr1 = meshes[1]->frame();

    static std::vector<Vec3_cu> verts0( 10000 );
    static std::vector<Vec3_cu> verts1( 10000 );
    verts0.resize( m0.get_nb_vertices() );
    verts1.resize( m1.get_nb_vertices() );

    srand( time(0) );

    for(int i = 0; i < m0.get_nb_vertices(); ++i)
        verts0[i] = (tr0 * m0.get_vertex(i).to_point());// + Vec3_cu::random(0.0001f);

    for(int i = 0; i < m1.get_nb_vertices(); ++i)
        verts1[i] = (tr1 * m1.get_vertex(i).to_point()) + Vec3_cu::random(0.0001f);

    verts1 = verts0; //////////////////////////////////////////////////////////////////////////////
#else

    if(g_mesh == 0 || g_mesh->_vbo.size() <= 0) return;

    const Mesh& m0 = *g_mesh;
    const Mesh& m1 = *g_mesh;

    static std::vector<Vec3_cu> verts0( 10000 );
    static std::vector<Vec3_cu> verts1( 10000 );
    verts0.resize( m0.get_nb_vertices() );
    verts1.resize( m1.get_nb_vertices() );

    srand( time(0) );

    Vec3_cu* pt = 0;

    g_mesh->_vbo.map_to(pt, GL_READ_ONLY);
    for(int i = 0; i < m0.get_nb_vertices(); ++i)
        verts0[i] = pt[i];// + Vec3_cu::random(0.0001f);
    g_mesh->_vbo.unmap();

    verts1 = verts0;
#endif

    GLLineWidthSave width( 5.f );
    GLPointSizeSave point(5.f);
    GLEnabledSave depth(GL_DEPTH_TEST, true, true);
    GLEnabledSave light(GL_LIGHTING, true, false);

    glPushMatrix();
    const float eps = 1.f - 0.001f;
    glTranslatef(p.x, p.y, p.z);
    glScalef(eps, eps, eps);
    glTranslatef(-p.x, -p.y, -p.z);

    glColor3f(1.f, 1.f, 0.f);
    glBegin(GL_LINES);
    int nb_inter = 0;
    for (int i = 0; i < m0.get_nb_tri(); ++i)
    {
        for (int j = 0; j < m1.get_nb_tri(); ++j)
        {
#if 1
            if(j == i) continue; // avoid testing a triangle on itself

            int a0[3] = { m0.get_tri(i*3+0), m0.get_tri(i*3+1), m0.get_tri(i*3+2) };
            int a1[3] = { m1.get_tri(j*3+0), m1.get_tri(j*3+1), m1.get_tri(j*3+2) };

            // Check for shared edge
            int acc = 0;
            for(int u = 0; u < 3; ++u) {
                for(int v = 0; v < 3; ++v) {
                    if( a0[u] == a1[v] ) acc++;
                }
            }
            if(acc > 1) continue;


            float r_eps = Cuda_ctrl::_debug._tab_vals[0];
            Vec3_cu p00 = verts0[a0[0]] + Vec3_cu::random(r_eps);
            Vec3_cu p01 = verts0[a0[1]] + Vec3_cu::random(r_eps);
            Vec3_cu p02 = verts0[a0[2]] + Vec3_cu::random(r_eps);

            Vec3_cu p10 = verts1[a1[0]] + Vec3_cu::random(r_eps);
            Vec3_cu p11 = verts1[a1[1]] + Vec3_cu::random(r_eps);
            Vec3_cu p12 = verts1[a1[2]] + Vec3_cu::random(r_eps);
/*

            Inter::Line l0[3] = { Inter::Line(p00-p01, p00), Inter::Line(p01-p02, p01), Inter::Line(p02-p00, p02) };
            Inter::Line l1[3] = { Inter::Line(p10-p11, p10), Inter::Line(p11-p12, p11), Inter::Line(p12-p10, p12) };
            // Check for shared edge
            int acc = 0;
            for(int u = 0; u < 3; ++u) {
                for(int v = 0; v < 3; ++v) {
                    Vec3_cu res;
                    if( Inter::line_line(l0[u], l1[v], res))
                        acc++;
                }
            }
            if(acc > 0) continue;
            */


#else
            // custom tri
            Vec3_cu p00(0.,0.,0.);
            Vec3_cu p01(1.,0.,0.5);
            Vec3_cu p02(0.,1.,0.);

            Vec3_cu p10(0.00001,0.,0.);
            Vec3_cu p11(-1.,0.,0.);
            Vec3_cu p12(0.,1.,0.);
#endif

            Inter::Triangle tri0(p00, p01, p02);
            Inter::Triangle tri1(p10, p11, p12);


            Vec3_cu res0, res1;
            bool cop = false;
            //t.start();
            bool res = false;
            //for(int i = 0; i < 1000000; ++i)
            {
#if 1
                res = Inter::tri_tri(tri0, tri1, res0, res1, cop, Cuda_ctrl::_debug._val0);
#else
                using namespace Cuda_utils;
                //const int block_size = 256;
                //const int grid_size = (d_input_vertices.size() + block_size - 1) / block_size;
                DA_bool r(1);
                DA_Vec3_cu p(2);

                tri_inter__(tri0, tri1, p.ptr(), p.ptr() + 1, r.ptr());


                res = r.fetch(0);
                res0 = p.fetch(0);
                res1 = p.fetch(1);
#endif
            }
            //std::cout << "Done in " << t.elapsed() << " sec" << std::endl;


            if(res && !cop && (res0-res1).norm() > Cuda_ctrl::_debug._val1)
            {
                // inter line:
                glColor3f(1.f, 1.f, 0.f);
//                res0 += Vec3_cu::random(0.05f);
//                res1 += Vec3_cu::random(0.05f);

                glVertex3f(res0.x, res0.y, res0.z);
                glVertex3f(res1.x, res1.y, res1.z);


            }
            // tri:
#if 0
            //if( acc < 2)
            {
                GLLineWidthSave width_( 2.f );
                Color::pseudo_rand(1).set_gl_state();
                glBegin(GL_LINES);
                for (int u = 0; u < 3; ++u) {
                    int c0 = u % 3;
                    int c1 = (u+1) % 3;
                    glVertex3f(tri0.p[c0].x, tri0.p[c0].y, tri0.p[c0].z);
                    glVertex3f(tri0.p[c1].x, tri0.p[c1].y, tri0.p[c1].z);
                }
                for (int u = 0; u < 3; ++u) {
                    int c0 = u % 3;
                    int c1 = (u+1) % 3;
                    glVertex3f(tri1.p[c0].x, tri1.p[c0].y, tri1.p[c0].z);
                    glVertex3f(tri1.p[c1].x, tri1.p[c1].y, tri1.p[c1].z);
                }
                glEnd();
                //goto exit;
            }

            nb_inter++;
#endif
            /*
                glBegin(GL_POINTS);
                glColor3f(1.f, 0.f, 0.f);
                glVertex3f(res0.x, res0.y, res0.z);
                glVertex3f(res1.x, res1.y, res1.z);
                glEnd();*/

        }
    }
exit:
    glEnd();
    glPopMatrix();
}

// -----------------------------------------------------------------------------

#if 0
template<class T>
__global__
void init(T** instance, T data)
{
    instance[0] = new T();
    instance[0] = data;
}

//il faut detruire  aussi !!!
template<class T>
T* malloc_device_heap(T* host_ptr)
{
    using namespace Cuda_utils;
    Device::Array<T > device_data(1);
    Device::Array<T*> instance(1);
    device_data[0] = host_ptr;
    init<<<1,1>>(instance.ptr(), device_data.ptr());

    return instance.fetch(0);
}

class Base : public Virtual_cu<Base> {

    Base(){

    }

    __device__ __host__
    virtual f();


    int _attr;
};


#endif


#endif // END FILE



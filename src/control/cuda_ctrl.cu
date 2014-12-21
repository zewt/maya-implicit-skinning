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
#include "cuda_ctrl.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "skeleton.hpp"
#include "port_glew.h"
#include "globals.hpp"
#include "cuda_globals.hpp"
#include "opengl_stuff.hpp"
#include "animesh.hpp"
#include "cuda_utils_common.hpp"
#include "constants.hpp"
#include "display_operator.hpp"
#include "endianess.hpp"
#include "cuda_main_kernels.hpp"
#include "skeleton_env.hpp"
#include "blending_env.hpp"

// =============================================================================
namespace Cuda_ctrl {
// =============================================================================

Path_ctrl            _paths;
Potential_plane_ctrl _potential_plane;
Animated_mesh_ctrl*  _anim_mesh = 0;
Skeleton_ctrl        _skeleton;
Display_ctrl         _display;
Debug_ctrl           _debug;
Graph_ctrl           _graph;
Operators_ctrl       _operators;
Color_ctrl           _color;

void load_mesh( Mesh* mesh )
{
    delete _anim_mesh;
    _anim_mesh = 0;
    delete g_animesh;
    g_animesh = 0;

    delete g_mesh;
    g_mesh = 0;

    g_mesh = mesh;

    //g_mesh->add_noise(5, 0.03f);
    g_mesh->center_and_resize(50.f);
    g_mesh->check_integrity();

    Color cl = _color.get(Color_ctrl::MESH_POINTS);
    g_mesh->set_point_color_bo(cl.r, cl.g, cl.b, cl.a);

    std::cout << "mesh loaded" << std::endl;
}

// -----------------------------------------------------------------------------

bool is_mesh_loaded(){
    return g_mesh != 0 && g_mesh->get_nb_vertices() != 0;
}

// -----------------------------------------------------------------------------

bool is_animesh_loaded(){
    return _anim_mesh != 0;
}

// -----------------------------------------------------------------------------

bool is_skeleton_loaded(){ return g_skel != 0; }

// -----------------------------------------------------------------------------

void erase_graph(){
    delete g_graph;
    g_graph = new Graph(g_mesh->get_offset(), g_mesh->get_scale());
}

// -----------------------------------------------------------------------------

void load_animesh_and_ssd_weights(const char* filename)
{
    delete g_animesh;
    g_animesh = new Animesh(g_mesh, g_skel);
    delete _anim_mesh;
    _anim_mesh = new Animated_mesh_ctrl(g_animesh);

    if(filename){
        // Check extension :
        int len = strlen(filename);
        bool has_commas = (filename[len-4] == '.') &
                (filename[len-3] == 'c') &
                (filename[len-2] == 's') &
                (filename[len-1] == 'v');
        std::cout << "reading weights\n";
        g_animesh->read_weights_from_file(filename, has_commas);
        std::cout << "weights ok" << std::endl;
    }

    g_animesh->update_base_potential();
}

// -----------------------------------------------------------------------------bool loadBlobScene( std::string file );
bool saveBlobScene( std::string file );
void addBlob();
bool removeBlob();
void selectAllBlobs();
void cleanBlobScene();
// -----------------------------------------------------------------------------
void setBlobRadius_selected( float r );
void setBlobRadius_all( float r );
void setBlobAlpha_selected( float a );
void setBlobAlpha_all( float a );
// -----------------------------------------------------------------------------
void setBlobOperatorType(std::string type);
void show_one( bool show );
// -----------------------------------------------------------------------------
void setRicciN(float n);
// -----------------------------------------------------------------------------
void setBlobCaniAlpha_selected( float a );
void setBlobCaniAlpha_all( float a );
void setBlobCaniW_selected( float w );
void setBlobCaniW_all( float w );
void use_cani_old_function( bool use );
// -----------------------------------------------------------------------------
void addEdge();
void deleteEdge();
void showWyvillGraph(bool show);
void addWyvillGroup( std::string type );
void deleteWyvillGroup(int group_idx);
void addSelectionToGroup(int group_idx);
void removeSelectionFromGroup(int group_idx);
// -----------------------------------------------------------------------------
void setBlobRestrictedBlendW_selected(float wA0A0);
void setBlobRestrictedBlendW_all(float wA0A0);
void setBlobRestrictedBlendInterW(float wA0A1, float wA1A0);
void use_deform_mk( bool use );

void load_animesh()
{
    delete g_animesh;
    g_animesh = new Animesh(g_mesh, g_skel);
    delete _anim_mesh;
    _anim_mesh = new Animated_mesh_ctrl(g_animesh);
}

void get_mem_usage(double& total, double& free)
{
    Cuda_utils::get_device_memory_usage(free, total);
}

// -----------------------------------------------------------------------------

void init_host()
{
    std::cout << "Initialize constants\n";
    Constants::init();
    std::cout << "Initialize endianness system\n";
    Endianess::init();

    std::cout << "Done\n";
}

// -----------------------------------------------------------------------------

void set_default_controller_parameters()
{
#if 0
    //for bulge-free blending skinning (elbow)
    Constants::set(Constants::F0, 1.f);
    Constants::set(Constants::F1, 0.43f);
    Constants::set(Constants::F2, 1.f);
    Constants::set(Constants::B0, 0.2f);
    Constants::set(Constants::B1, 0.7f);
    Constants::set(Constants::B2, 1.2f);
    Constants::set(Constants::POW0, 1.f);
    Constants::set(Constants::POW1, 1.f);
#else
    Constants::set(Constants::F0, 0.5f );
    Constants::set(Constants::F1, 0.5f );
    Constants::set(Constants::F2, 0.5f );
    Constants::set(Constants::B0, 0.2f );
    Constants::set(Constants::B1, 0.7f );
    Constants::set(Constants::B2, 1.2f );
    Constants::set(Constants::POW0, 1.f);
    Constants::set(Constants::POW1, 1.f);
#endif
    //Blending_env::update_opening(); <- // TODO: to be deleted
    //Blending_env::set_global_ctrl_shape(shape);
}

// -----------------------------------------------------------------------------

GLuint init_tex_operator( int width, int height )
{
    float* tex = compute_tex_operator(width, height,
                                      Cuda_ctrl::_display._operator_type,
                                      Cuda_ctrl::_display._operator_mode,
                                      Cuda_ctrl::_display._opening_angle,
                                      Cuda_ctrl::_display._custom_op_id );

    glAssert( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    GLuint tex_id;
    glAssert( glGenTextures(1, &tex_id) );
    glAssert( glBindTexture(GL_TEXTURE_2D, tex_id) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST) );

    //printf("tex ptr %d\n", tex);
    glAssert( glTexImage2D(GL_TEXTURE_2D,
                           0,
                           GL_RGBA,
                           width,
                           height,
                           0,
                           GL_RGBA,
                           GL_FLOAT,
                           tex) );

    glAssert( glBindTexture(GL_TEXTURE_2D, 0) );
    delete[] tex;
    return tex_id;
}

// -----------------------------------------------------------------------------

void init_opengl_cuda()
{
    // NOTE this function should not call ANY cuda API functions
    g_mesh  = new Mesh();
    g_graph = new Graph(g_mesh->get_offset(), g_mesh->get_scale());

    g_op_tex = init_tex_operator(100, 100);
}

// -----------------------------------------------------------------------------

void cuda_start(const std::vector<Blending_env::Op_t>& op)
{
    using namespace Cuda_ctrl;

#ifndef NDEBUG
    std::cout << "WARNING: you're still in debug mode" << std::endl;
#endif
    init_host();
    init_cuda( op );

    set_default_controller_parameters();

    atexit(cleanup);
}

// -----------------------------------------------------------------------------

void cleanup()
{
    cudaDeviceSynchronize();
    CUDA_CHECK_ERRORS();

    // OpenGL ---------------
    glAssert( glBindTexture(GL_TEXTURE_2D, 0) );
    glAssert( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    glAssert( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0) );
    glUseProgram( 0 );

    glAssert( glDeleteTextures(1, &g_op_frame_tex) );
    glAssert( glDeleteTextures(1, &g_op_tex) );

    // End OpenGL ----------

    Constants::free();

    delete g_skel; // Skeleton must be deleted before blending env
    delete g_animesh;
    delete g_graph;
    delete g_mesh;

    g_skel        = 0;
    g_animesh     = 0;
    g_graph       = 0;
    g_mesh        = 0;

    Blending_env::clean_env();
    HRBF_env::clean_env();
    Precomputed_env::clean_env();
    Skeleton_env::clean_env();

    CUDA_CHECK_ERRORS();

    cudaDeviceReset();
}

}// END CUDA_CTRL NAMESPACE  ===================================================

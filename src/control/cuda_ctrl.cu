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
#include "globals.hpp"
#include "cuda_globals.hpp"
#include "animesh.hpp"
#include "cuda_utils_common.hpp"
#include "constants.hpp"
#include "endianess.hpp"
#include "cuda_main_kernels.hpp"
#include "skeleton_env.hpp"
#include "blending_env.hpp"

// =============================================================================
namespace Cuda_ctrl {
// =============================================================================

Animated_mesh_ctrl*  _anim_mesh = 0;
Skeleton_ctrl        _skeleton;
Debug_ctrl           _debug;
Graph_ctrl           _graph;
Operators_ctrl       _operators;

void load_mesh( Mesh* mesh )
{
    delete _anim_mesh;
    _anim_mesh = 0;
    delete g_animesh;
    g_animesh = 0;

    delete g_mesh;
    g_mesh = 0;

    g_mesh = mesh;

    g_mesh->check_integrity();

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

bool is_skeleton_loaded(){ return _skeleton.is_loaded(); }

// -----------------------------------------------------------------------------

void erase_graph(){
    delete g_graph;
    g_graph = new Graph(g_mesh->get_offset(), g_mesh->get_scale());
}

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

void init_opengl_cuda()
{
    // NOTE this function should not call ANY cuda API functions
    g_mesh  = new Mesh();
    g_graph = new Graph(g_mesh->get_offset(), g_mesh->get_scale());
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

//    atexit(cleanup);
}

// -----------------------------------------------------------------------------

void cleanup()
{
    cudaDeviceSynchronize();
    CUDA_CHECK_ERRORS();

    Constants::free();

    _skeleton.cleanup(); // Skeleton must be deleted before blending env
    delete g_animesh;
    delete g_graph;
    delete g_mesh;

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

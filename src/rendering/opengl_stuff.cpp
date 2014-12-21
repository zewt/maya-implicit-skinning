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
#include "globals.hpp"
#include "cuda_gl_interop_wrapper.hpp"
#include "opengl_stuff.hpp"
#include "glsave.hpp"
#include "cuda_ctrl.hpp"
#include "blending_env_tex_interface.hpp"

#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// -----------------------------------------------------------------------------

void initShaders(Shader_prog** shader_,
                 const std::string& vs_source_name,
                 const std::string& fs_source_name)
{
    Shader vs(vs_source_name, GL_VERTEX_SHADER);
    Shader fs(fs_source_name, GL_FRAGMENT_SHADER);
    *shader_ = new Shader_prog(vs, fs);
    (*shader_)->link();
}

// -----------------------------------------------------------------------------

void erase_shaders()
{
    delete g_points_shader;
}

// -----------------------------------------------------------------------------

void load_shaders()
{
    erase_shaders();

    g_points_shader = new Shader_prog();
    Shader vertex_s("./src/shaders/points.vert", GL_VERTEX_SHADER);
    Shader geom_s("./src/shaders/points.geom", GL_GEOMETRY_SHADER);
    g_points_shader->set_shader(vertex_s);
    g_points_shader->set_shader(geom_s);

    GLuint sid = g_points_shader->get_id();
    glProgramParameteriEXT(sid, GL_GEOMETRY_INPUT_TYPE_EXT, GL_POINTS);
    glProgramParameteriEXT(sid, GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP);
    glProgramParameteriEXT(sid, GL_GEOMETRY_VERTICES_OUT_EXT, 4);
    g_points_shader->link();
}

// -----------------------------------------------------------------------------

void init_shaders_ptr_to_zero()
{
    g_points_shader = 0;
}

// -----------------------------------------------------------------------------

void print_opengl_infos()
{
    printf("\n --- OPENGL INFOS ---\n");
    printf("Implementation vendor : %s\n", glGetString(GL_VENDOR));
    printf("Renderer : %s\n", glGetString(GL_RENDERER));
    printf("Opengl version : %s\n", glGetString(GL_VERSION));
    printf("Shader version : %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    //printf("Supported extensions : %s\n", glGetString(GL_EXTENSIONS));
    int n;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n);
    printf("maximum attributes per vertex : %d\n", n);
    printf("\n --- END OPENGL INFOS ---\n");
    printf("\n");
    fflush(stdout);
}

// -----------------------------------------------------------------------------

void init_opengl()
{
    //test();
    print_opengl_infos();
    init_shaders_ptr_to_zero();
    load_shaders();

    //Generate array buffer for the quad
    glAssert( glGenBuffers(1, &g_gl_quad) );
    const GLfloat display_quad[16]= { 1.f, 1.f, 1.f, 1.f,
                                      1.f,-1.f, 1.f, 0.f,
                                      -1.f,-1.f, 0.f, 0.f,
                                      -1.f, 1.f, 0.f, 1.f};
    //Copy data to array buffer
    glAssert( glBindBuffer(GL_ARRAY_BUFFER, g_gl_quad) );
    glAssert( glBufferData(GL_ARRAY_BUFFER, 16*sizeof(GLfloat), display_quad, GL_STATIC_DRAW) );

    glAssert( glActiveTexture(GL_TEXTURE0) );
    // gen pbo

    // FIXME: screen texture should be allocated in an independant context
    const int width  = 0;//Cuda_ctrl::_display._width;
    const int height = 0;//Cuda_ctrl::_display._height;

    //Generate textures
    GLEnabledSave save_tex(GL_TEXTURE_2D, true, true);
    glAssert( glEnable(GL_DEPTH_TEST) );


    glAssert( glGenTextures(NB_TEX, g_gl_Tex) );

    glAssert( glBindTexture(GL_TEXTURE_2D, g_gl_Tex[COLOR]) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    ////////////////////
    glAssert( glTexImage2D(GL_TEXTURE_2D,
                           0,
                           GL_RGBA,
                           width,
                           height,
                           0,
                           GL_RGBA,
                           GL_UNSIGNED_BYTE,
                           0) );
    ////////////////////////////

    glAssert( glBindTexture(GL_TEXTURE_2D, g_gl_Tex[DEPTH]) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    ////////////////////
    glAssert( glTexImage2D(GL_TEXTURE_2D,
                           0,                      // Mimap level
                           GL_DEPTH_COMPONENT24,   // internalFormat
                           width,                  // Width
                           height,                 // Height
                           0,                      // border zise
                           GL_DEPTH_COMPONENT,     // format
                           GL_UNSIGNED_BYTE,       // type
                           0                       // data (current tex binding)
                           ));
    /////////////////

    glAssert( glBindTexture(GL_TEXTURE_2D, g_gl_Tex[NORMAL_MAP]) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );
    ////////////////////
    glAssert( glTexImage2D(GL_TEXTURE_2D,
                           0,
                           GL_RGBA,
                           width,
                           height,
                           0,
                           GL_RGBA,
                           GL_UNSIGNED_BYTE,
                           0) );
    ////////////////////////////

    glActiveTexture(GL_TEXTURE0);
    ////////////////////////////

    glAssert( glBindTexture(GL_TEXTURE_2D, g_gl_Tex[MAP]) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    //controller frame
    glAssert( glGenTextures(1, &g_ctrl_frame_tex) );
    glAssert( glBindTexture(GL_TEXTURE_2D, g_ctrl_frame_tex) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    glAssert( glBindTexture(GL_TEXTURE_2D, 0) );

    glAssert( glEnable(GL_LINE_SMOOTH) );

    // Init some basic geometric meshes into GPU with vertex buffer objects
    g_sphere_lr_vbo      = g_primitive_printer.init_sphere(1.f, 10);
    g_sphere_vbo         = g_primitive_printer.init_sphere(1.f, 100);
    g_circle_vbo         = g_primitive_printer.init_circle(1.f, 50);
    g_arc_circle_vbo     = g_primitive_printer.init_arc_circle(1.f, 25, M_PI);
    g_circle_lr_vbo      = g_primitive_printer.init_circle(1.f, 10);
    g_arc_circle_lr_vbo  = g_primitive_printer.init_arc_circle(1.f, 5, M_PI);
    g_grid_vbo           = g_primitive_printer.init_grid(1.f, 1.f, 25, 25);
    g_cylinder_vbo       = g_primitive_printer.init_cylinder(1.f, 1.f, 50, 50);
    g_cylinder_cage_vbo  = g_primitive_printer.init_cylinder_cage(1.f, 1.f, 200, 6);
    g_cube_vbo           = g_primitive_printer.init_cube();
}

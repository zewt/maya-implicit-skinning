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
#include "ppm_loader.hpp"
#include "glshoot.hpp"
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

/// Activate shader program
void EnableProgram()
{
    GLuint sid = g_dummy_quad_shader->get_id();
    GLint vertex_attrib = glGetAttribLocation(sid,"vertex_coord");
    glAssert( glEnableVertexAttribArray(vertex_attrib) );
    g_dummy_quad_shader->use();
}

// -----------------------------------------------------------------------------

/// Deactivate shader program
void DisableProgram()
{
    GLuint sid = g_dummy_quad_shader->get_id();
    GLint vertex_attrib = glGetAttribLocation(sid,"vertex_coord");
    glAssert( glDisableVertexAttribArray(vertex_attrib) );
    glUseProgram(0);
}

// -----------------------------------------------------------------------------

void init_phong_shaders()
{
    std::stringstream out;
    out << Tex_units::KD;
    std::string unit_kd   = out.str();
    out << Tex_units::BUMP;
    std::string unit_bump = out.str();
    typedef std::pair<std::string, std::string> Pair_str;
    std::vector<Pair_str> shader_symbols[NB_PHONG_SHADERS];
    shader_symbols[MAP_KD     ].push_back(Pair_str("TEXTURE_KD"  , unit_kd)  );
    shader_symbols[MAP_KD_BUMP].push_back(Pair_str("TEXTURE_KD"  , unit_kd)  );
    shader_symbols[MAP_KD_BUMP].push_back(Pair_str("TEXTURE_BUMP", unit_bump));

    for(int i = 0; i < NB_PHONG_SHADERS; i++)
    {
        Shader vs(GL_VERTEX_SHADER);
        Shader gs(GL_GEOMETRY_SHADER);
        Shader fs(GL_FRAGMENT_SHADER);

        for(unsigned j = 0; j < shader_symbols[i].size(); j++)
        {
            Pair_str p = shader_symbols[i][j];
            vs.add_define(p.first, p.second);
            gs.add_define(p.first, p.second);
            fs.add_define(p.first, p.second);
        }
        vs.load_file("./src/shaders/phong.vert");
        gs.load_file("./src/shaders/phong.geom");
        fs.load_file("./src/shaders/phong.frag");

        g_phong_list[i] = new Shader_prog();
        g_phong_list[i]->set_shader(vs);
        g_phong_list[i]->set_shader(gs);
        g_phong_list[i]->set_shader(fs);

        int prog_id = g_phong_list[i]->get_id();
        glProgramParameteriEXT(prog_id, GL_GEOMETRY_INPUT_TYPE_EXT, GL_TRIANGLES);
        glProgramParameteriEXT(prog_id, GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP);
        glProgramParameteriEXT(prog_id, GL_GEOMETRY_VERTICES_OUT_EXT, 256);

        g_phong_list[i]->link();
    }
}

// -----------------------------------------------------------------------------

void erase_shaders()
{
    delete g_ssao_shader;
    delete g_normal_map_shader;
    delete g_dummy_quad_shader;
    delete g_lavalamp_program;
    delete g_lavalamp_body_program;
    delete g_arrows_program;
    delete g_points_shader;

    for(int i = 0; i < NB_PHONG_SHADERS; i++) delete g_phong_list[i];
}

// -----------------------------------------------------------------------------

void load_shaders()
{
    erase_shaders();

    initShaders(&g_ssao_shader,"./src/shaders/ssao.vert","./src/shaders/ssao.frag");
    initShaders(&g_normal_map_shader,"./src/shaders/normal_map.vert","./src/shaders/normal_map.frag");
    initShaders(&g_dummy_quad_shader,"./src/shaders/dummy_quad.vert","./src/shaders/dummy_quad.frag");
    initShaders(&g_lavalamp_program,"./src/shaders/lavalamp.vert","./src/shaders/lavalamp.frag");
    initShaders(&g_lavalamp_body_program,"./src/shaders/lavalampbody.vert","./src/shaders/lavalampbody.frag");
    initShaders(&g_arrows_program,"./src/shaders/arrows.vert","./src/shaders/arrows.frag");

    init_phong_shaders();

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
    g_ssao_shader = g_normal_map_shader = g_dummy_quad_shader = g_lavalamp_program =
    g_lavalamp_body_program = g_arrows_program = g_points_shader = 0;;

    for(int i = 0; i < NB_PHONG_SHADERS; i++) g_phong_list[i] = 0;
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

    //Since the quad is the only geometry drawn, we set the vertex
    //and texture pointer once and for all
    g_dummy_quad_shader->use();
    GLuint sid = g_dummy_quad_shader->get_id();
    GLint vertex_attrib = glGetAttribLocation (sid,"vertex_coord");
    GLint tex_sampler   = glGetUniformLocation(sid,"tex_sampler");
    glAssert( glUniform1i(tex_sampler,0) );
    glAssert( glEnableVertexAttribArray(vertex_attrib) );
    glAssert( glVertexAttribPointer(vertex_attrib,2,GL_FLOAT,GL_FALSE,4*sizeof(GLfloat),NULL) );
    glAssert( glTexCoordPointer(2,GL_FLOAT,4*sizeof(GLfloat),((char*)NULL + 2*sizeof(GLfloat))) );
    glAssert( glBindBuffer(GL_ARRAY_BUFFER,0) );
    glAssert( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0) );
    glAssert( glDisableVertexAttribArray(vertex_attrib) );
    Shader_prog::unuse();

    g_lavalamp_program->use();
    sid = g_lavalamp_program->get_id();
    GLint lavalamp_tex_sampler = glGetUniformLocation(sid,"tex_sampler");
    glAssert( glUniform1i(lavalamp_tex_sampler,0) );
    lavalamp_tex_sampler = glGetUniformLocation(sid,"tex_sampler");
    glAssert( glUniform1i(lavalamp_tex_sampler,0) );
    Shader_prog::unuse();

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

    glActiveTexture(GL_TEXTURE2);////////////////////////////////////////
    glAssert( glBindTexture(GL_TEXTURE_2D, g_gl_Tex[NOISE]) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    ////////////////////
    int* noise_ptr = 0;
    int some_x, some_y;
    if( !Ppm_loader::read("./resource/textures/noise.ppm", some_x, some_y, noise_ptr) ){
        printf("Not loading noise texture\n");
    } else {
        glAssert( glTexImage2D(GL_TEXTURE_2D,
                               0,
                               GL_RGBA,
                               some_x,
                               some_y,
                               0,
                               GL_RGBA,
                               GL_UNSIGNED_BYTE,
                               noise_ptr) );
    }
    delete[] noise_ptr;
    noise_ptr = 0;
    glActiveTexture(GL_TEXTURE0);
    ////////////////////////////

    glAssert( glBindTexture(GL_TEXTURE_2D, g_gl_Tex[MAP]) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    int* img_envmap = 0;
    int envmapx, envmapy;
    if( !Ppm_loader::read("./resource/textures/skymap.ppm", envmapx, envmapy, img_envmap) ){
        printf("Not loading skymap texture\n");
    } else {
        glAssert( glTexImage2D(GL_TEXTURE_2D,
                               0,
                               GL_RGBA,
                               envmapx,
                               envmapy,
                               0,
                               GL_RGBA,
                               GL_UNSIGNED_BYTE,
                               img_envmap) );
    }
    delete[] img_envmap;
    img_envmap = 0;

    //controller frame
    glAssert( glGenTextures(1, &g_ctrl_frame_tex) );
    glAssert( glBindTexture(GL_TEXTURE_2D, g_ctrl_frame_tex) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );
    int* frame_img = 0;
    int framex, framey;
    if( !Ppm_loader::read_with_alpha("resource/textures/controller_frame.ppm", framex, framey, frame_img) ){
        printf("error loading controller frame\n");
    } else {
        glAssert( glTexImage2D(GL_TEXTURE_2D,
                               0,
                               GL_RGBA,
                               framex,
                               framey,
                               0,
                               GL_RGBA,
                               GL_UNSIGNED_BYTE,
                               frame_img) );
    }
    free(frame_img);
    frame_img = 0;

    glAssert( glGenTextures(1, &g_op_frame_tex) );
    glAssert( glBindTexture(GL_TEXTURE_2D, g_op_frame_tex) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
    glAssert( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) );

    if( !Ppm_loader::read_with_alpha("resource/textures/operator_frame.ppm", framex, framey, frame_img)) {
        printf("error loading operator frame\n");
    } else {
        glAssert( glTexImage2D(GL_TEXTURE_2D,
                               0,
                               GL_RGBA,
                               framex,
                               framey,
                               0,
                               GL_RGBA,
                               GL_UNSIGNED_BYTE,
                               frame_img) );
    }
    free(frame_img);
    frame_img = 0;

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

// -----------------------------------------------------------------------------

void draw_quad()
{
    glColor4f(1.f, 1.f, 1.f, 1.f);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    //glOrtho(0.f,1.f,0.f,1.f,0.f,1.f);

    glBegin(GL_QUADS);
    glTexCoord2f(0.f,0.f);
    glVertex2f(-1.f,-1.f);
    glTexCoord2f(1.f,0.f);
    glVertex2f(1.f,-1.f);
    glTexCoord2f(1.f,1.f);
    glVertex2f(1.f,1.f);
    glTexCoord2f(0.f,1.f);
    glVertex2f(-1.f,1.f);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// -----------------------------------------------------------------------------

void draw_operator(int x, int y, int w, int h)
{
    glDisable(GL_DEPTH_TEST);
    GLEnabledSave save_tex(GL_TEXTURE_2D, true, true);
    glBindTexture(GL_TEXTURE_2D, g_op_tex);
    GLViewportSave save_viewport;
    glViewport(x,y,w,h);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.f,1.f,0.f,1.f,0.f,1.f);

    glColor4f(1.f, 1.f, 1.f, 1.f);
    //glColor4f(150.f, 150.f, 150.f, 1.f);

    float x0 = 0.125f, x1 = 0.89f;
    float y0 = 0.135f, y1 = 0.9f;
    GLTexEnvModeSave save_tex_env;
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    glBegin(GL_QUADS);
    glTexCoord2f(0.f,0.f);
    glVertex2f(x0, y0);
    glTexCoord2f(1.f,0.f);
    glVertex2f(x1, y0);
    glTexCoord2f(1.f,1.f);
    glVertex2f(x1, y1);
    glTexCoord2f(0.f,1.f);
    glVertex2f(x0 ,y1);
    glEnd();
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindTexture(GL_TEXTURE_2D, g_op_frame_tex);
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

    glDisable (GL_BLEND);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// -----------------------------------------------------------------------------

void draw_global_controller(int x, int y, int w, int h)
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
    for(int i = 0; i < 100; i++){
        float t0 = i * 0.01f;
        float t1 = (i+1) * 0.01f;
        float f0 = Blending_env::eval_global_ctrl(cosf(t0 * M_PI));
        float f1 = Blending_env::eval_global_ctrl(cosf(t1 * M_PI));
        t0 = t0 * (x1 - x0) + x0; t1 = t1 * (x1 - x0) + x0;
        f0 = f0 * (y1 - y0) + y0; f1 = f1 * (y1 - y0) + y0;
        glVertex2f(t0,f0);
        glVertex2f(t1,f1);
    }
    glEnd();
    glDisable (GL_BLEND);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// -----------------------------------------------------------------------------

void draw_circle(int width, int height, int x, int y, float rad)
{
    GLEnabledSave save_tex(GL_TEXTURE_2D, true, false);
    GLEnabledSave save_light(GL_LIGHTING, true, false);
    GLLineWidthSave line_save( 0.8f );

    glAssert( glMatrixMode(GL_PROJECTION) );
    glAssert( glPushMatrix() );
    glAssert( glLoadIdentity() );
    glAssert( glOrtho(0.f, (GLfloat)width, 0.f, (GLfloat)height, 0.f, 1.f) );

    {
        glAssert( glMatrixMode(GL_MODELVIEW) );
        glAssert( glPushMatrix() );
        glAssert( glLoadIdentity() );

        glAssert( glTranslatef((GLfloat)x, (GLfloat)(height - y), 0.f) );

        glAssert( glScalef(rad, rad, rad) );
        g_primitive_printer.draw(g_circle_vbo);
        glAssert( glPopMatrix() );
    }

    glAssert( glMatrixMode(GL_PROJECTION) );
    glAssert( glPopMatrix() );
    glAssert( glMatrixMode(GL_MODELVIEW) );

}

// -----------------------------------------------------------------------------

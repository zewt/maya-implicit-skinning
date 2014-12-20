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
#include "cuda_rendering.hpp"

#include "gltex2D.hpp"
#include "animesh.hpp"
#include "glbuffer_object.hpp"
#include "depth_peeling.hpp"
#include "opengl_stuff.hpp"
#include "cuda_globals.hpp"
#include "cuda_stuff.hpp"
#include "cuda_ctrl.hpp"
#include "skeleton.hpp"
#include "filters.hpp"
#include "globals.hpp"
#include "scene.hpp"
#include "conversions.hpp"

#include "cuda_main_kernels.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

// Forward def "skeleton_env.hpp" ----------------------------------------------
namespace Skeleton_env {
void draw_grids();
}
// END Forward def "skeleton_env.hpp" ------------------------------------------

// -----------------------------------------------------------------------------

void draw_controller()
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();
    if(set.size() > 0)
    {
        int bone_id = set[set.size()-1];
        int pt = g_skel->parent( bone_id );
        if( pt > -1 ){
            Blending_env::Ctrl_id ctrl_id = g_skel->get_ctrl(/*bone_id*/ pt);
            draw_controller(ctrl_id, 0, 0, 320, 240);
        }
    }
    else
    {

        glColor3f(0.f, 1.f, 0.f);
        draw_global_controller(0  , 0, 320, 240);

    }
}

// -----------------------------------------------------------------------------

static void draw_junction_sphere(bool rest_pose)
{
    if(g_skel == 0 || !Cuda_ctrl::_display._junction_spheres)
        return;

    using namespace Cuda_ctrl;
    glColor4f(0.4f, 0.3f, 1.f, 0.5f);
    const std::vector<int>& set = _skeleton.get_selection_set();
    for(unsigned i = 0; i < set.size(); i++)
    {
        float r  = g_animesh->get_junction_radius(set[i]);
        Vec3_cu v = rest_pose ? g_skel->joint_rest_pos(set[i]) : g_skel->joint_pos(set[i]);
        glPushMatrix();
        glTranslatef(v.x, v.y, v.z);
        glScalef(r, r, r);
        g_primitive_printer.draw(g_sphere_vbo);
        glPopMatrix();
    }
}

// -----------------------------------------------------------------------------

static void draw_cylinder()
{
    using namespace Cuda_ctrl;
    const std::vector<int>& selection_set = _skeleton.get_selection_set();

    for(unsigned int i = 0; i<selection_set.size(); i++)
    {
        int id = selection_set[i];
        if( g_skel->bone_type(id) == EBone::CYLINDER )
        {
            const Bone* b = g_skel->get_bone(id);
            float rad = b->radius();
            glMatrixMode(GL_MODELVIEW);

            glPushMatrix();
            Transfo tr_trans = b->get_frame().transpose();
            glMultMatrixf(tr_trans.m);
            glRotatef(90.f, 0.f, 1.f, 0.f);
            glScalef(rad, rad, b->length());
            g_primitive_printer.draw(g_cylinder_cage_vbo);
            glPopMatrix();
        }

    }
}

// -----------------------------------------------------------------------------

static void draw_hrbf_points(bool rest_pose)
{
    using namespace Cuda_ctrl;

    if(_display._draw_hrbf_samples ||
        _display._edit_hrbf_samples)
    {
        const std::vector<int>& selection_set = _skeleton.get_selection_set();
        if(_anim_mesh != 0)
            _anim_mesh->draw_hrbf_points(selection_set, true, rest_pose);
    }
}

// -----------------------------------------------------------------------------

static void draw_mesh_points(const Camera* cam, bool rest_pose)
{
    if(Cuda_ctrl::_anim_mesh == 0) return;

    if(Cuda_ctrl::_anim_mesh->is_point_displayed())
    {
        // Do a little offset so mesh points won't hide rbf samples
        const float eps = 1.0001f;
        glPushMatrix();
        Vec3_cu p = cam->get_pos();
        glTranslatef(p.x, p.y, p.z);
        glScalef(eps, eps, eps);
        glTranslatef(-p.x, -p.y, -p.z);

        glPointSize(9.f);
        GLEnabledSave save_point (GL_POINT_SMOOTH, true, true );
        rest_pose ? g_animesh->draw_points_rest_pose() : g_mesh->draw_points();
        glPopMatrix();
    }
}

// -----------------------------------------------------------------------------

static void draw_grid_lines(const Camera* cam)
{
    if( Cuda_ctrl::_display._grid)
    {
        GLLineWidthSave line_width_save;

        glColor3f(1.f, 1.f, 1.f);
        glPushMatrix();
        if( cam->is_ortho() )
        {
            // When view is aligned with world axis we show the grid
            float threshold = 1.f - 0.000001f;
            if( fabsf(cam->get_dir().dot(Vec3_cu::unit_x()) ) > threshold)
                glRotatef(90.f, 0.f, 1.f, 0.f);
            else if( fabsf(cam->get_dir().dot(Vec3_cu::unit_y()) ) > threshold)
                glRotatef(90.f, 1.f, 0.f, 0.f);

            //float w = cam->get_frustum_width();
            glLineWidth(1.f);
            //glScalef(w, w, w);
            g_primitive_printer.draw(g_grid_vbo);
        }
        else
        {
            glLineWidth(2.f);
            glScalef(30.f, 30.f, 30.f);
            g_primitive_printer.draw(g_grid_vbo);
        }
        glPopMatrix();
    }
}

// -----------------------------------------------------------------------------

void draw_normals(const std::vector<int>& selected_points,
                  const Cuda_utils::DA_Vec3_cu& d_ssd_normals)
{
    Vec3_cu* vert = 0;

    g_mesh->_vbo.map_to(vert, GL_READ_ONLY);

    glBegin(GL_LINES);
    glColor4f(1.f, 0.f, 0.f, 1.f);
    for(unsigned i = 0; i < selected_points.size(); i++)
    {
        Vec3_cu n;
        d_ssd_normals.fetch(selected_points[i], n);
        n.normalize();
        n = n*5.f;

        const Mesh::Packed_data d = g_mesh->get_packed_vert_map()[selected_points[i]];
        Vec3_cu    v     = vert[ d.idx_data_unpacked ];

        //glColor3f(n.x, n.y, n.z);
        glVertex3f(v.x, v.y, v.z);
        glVertex3f(v.x + n.x, v.y + n.y, v.z + n.z);

    }
    glEnd();

    g_mesh->_vbo.unmap();
}

// -----------------------------------------------------------------------------

// TODO: re-write this with using context
void redraw_with_ssao(/* Render_context_cu* ctx */)
{
#if 0
    const int width  = Cuda_ctrl::_display._width;
    const int height = Cuda_ctrl::_display._height;

    ssao_shader->use();
    GLEnabledSave texture_2d(GL_TEXTURE_2D, true, true);
    GLActiveTexUnitSave tex_unit_save;

    glActiveTexture(GL_TEXTURE0);
    glAssert( glBindTexture(GL_TEXTURE_2D, gl_Tex[NORMAL_MAP]) );
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     0, 0, width, height, 0);

    ssao_shader->set_uniform("bgl_RenderedTexture", 0);


    glActiveTexture(GL_TEXTURE2);
    glAssert( glBindTexture(GL_TEXTURE_2D, gl_Tex[DEPTH]) );
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                     0, 0, width, height, 0);

    ssao_shader->set_uniform("bgl_DepthTexture", 2);

    int v = Cuda_ctrl::_display._width;
    ssao_shader->set_uniform("width", v);
    v = Cuda_ctrl::_display._height;
    ssao_shader->set_uniform("height", v);

    draw_quad();
    ShaderProgram::unuse();
#endif
}

// -----------------------------------------------------------------------------

void draw_animesh(bool use_color_array, bool use_point_color, bool rest_pose)
{
    if(g_animesh == 0) return;

    if(rest_pose) g_animesh->draw_rest_pose(use_color_array, use_point_color);
    else          g_animesh->draw(use_color_array, use_point_color);
}

// -----------------------------------------------------------------------------

static void draw_skeleton(const Camera* cam, bool draw_skel, bool rest_pose)
{
    using namespace Cuda_ctrl;
    std::vector<int> joints = _skeleton.get_selection_set();
    int              node   = _graph.get_selected_node();

    if     ( draw_skel && g_skel != 0 ) g_skel ->draw( *cam, joints, rest_pose);
    else if( g_graph != 0 )             g_graph->draw( *cam, node );
}

// -----------------------------------------------------------------------------

static void draw_voxels()
{
    if(g_animesh != 0)
    {
        if( g_animesh->get_vox_mesh() != 0 &&
            Cuda_ctrl::_display._draw_voxels)
        {
            GLPolygonModeSave save_poly_mode;
            glAssert( glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) );
            glColor3f(1.f, 0.f, 0.f);
            g_animesh->get_vox_mesh()->draw();
        }
    }
}

// -----------------------------------------------------------------------------

/// what is drawn here will be hidden by the transceluscent mesh
/// Hardware Antialiasing will work
static void plain_objects(const Camera* cam, const Render_context_cu* ctx,
                          float r, float g, float b, float bfactor)
{
    using namespace Cuda_ctrl;
    Cuda_ctrl::_potential_plane.draw();

    if( _display._wire )
    {
        glAssert( glColor4f(r, g, b, bfactor) );
        if( !ctx->_skeleton )
        {
            if ( _anim_mesh != 0 ) draw_animesh(true, true, true /*rest pose*/);
            else if( g_mesh != 0 ) g_mesh->draw(true, true);
        }
        else
        {
            if(ctx->_draw_mesh) draw_animesh(true, true, ctx->_rest_pose);
        }
    }

    if( !ctx->_plain_phong )
    {
        draw_hrbf_points(ctx->_rest_pose);
        draw_mesh_points(cam, ctx->_rest_pose);
        draw_grid_lines(cam);

        draw_cylinder();
        draw_voxels();

        if(_debug._show_normals)
            draw_normals(_anim_mesh->get_selected_points(),
                         g_animesh->get_ssd_normals());

        if(_debug._show_gradient)
            _debug.draw_gradient( _anim_mesh->get_selected_points(),
                                 g_animesh->get_gradient().ptr());

    }

}

// -------------------------------------------------------------------------

/// What follows won't benefit from hardware AA, but will be visible in
/// transluscent mode
static void transparent_objects(const Camera* cam, const Render_context_cu* ctx)
{
    using namespace Cuda_ctrl;
    Cuda_ctrl::_potential_plane.draw();


    if( !ctx->_skeleton )
    {
        if     ( _anim_mesh != 0 ) draw_animesh(true, false, true /*rest pose*/);
        else if( g_mesh     != 0 ) g_mesh->draw();
    }
    else
    {
        if(ctx->_draw_mesh) draw_animesh(true, false, ctx->_rest_pose);
    }


    if( !ctx->_plain_phong  && Cuda_ctrl::_skeleton.is_displayed())
    {
        if(_anim_mesh != 0)
            Cuda_ctrl::_anim_mesh->draw_rotation_axis();

        glAssert( glColor4f(1.f, 0.f, 0.f, 1.f) );
        draw_junction_sphere(ctx->_rest_pose);
        draw_skeleton(cam, ctx->_skeleton, ctx->_rest_pose);
    }
}


// Class RenderFuncWireframe ===================================================

void RenderFuncWireframe::draw_transc_objs()
{
    transparent_objects(_cam, _ctx);
}

// -----------------------------------------------------------------------------

void RenderFuncWireframe::f()
{
    //glPolygonOffset(1.1f, 4.f);
    //glEnable(GL_POLYGON_OFFSET_FILL);
    transparent_objects(_cam, _ctx);
    //glDisable(GL_POLYGON_OFFSET_FILL);
}

// -----------------------------------------------------------------------------

void RenderFuncWireframe::render(const Camera* cam,
                                 const Render_context_cu* ctx)
{
    GLEnabledSave save_light(GL_LIGHTING, true, false);
    glAssert( glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) );
    glAssert( glLineWidth(1.f) );
    glAssert( glHint(GL_LINE_SMOOTH_HINT, GL_NICEST) );
    glAssert( glEnable(GL_LINE_SMOOTH) );

    glAssert( glBlendFunc(GL_DST_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
    //plain_objects(cam, 0.1f, 0.15f, 0.7f, 1.f);
    plain_objects(cam, ctx, 1.f, 1.f, 1.f, 1.f);
    glAssert( glDisable(GL_LINE_SMOOTH) );
    glAssert( glPolygonMode(GL_FRONT_AND_BACK, GL_FILL) );
}

// END RENDER_FUNC_WIRE_FRAME ==================================================

// CLASS Render_context_cu =====================================================

Render_context_cu::Render_context_cu(int w, int h):
    _pbo_color(0),
    _pbo_depth(0),
    _d_img_buffer(0),
    _d_bloom_buffer(0),
    _d_rendu_buf(0),
    _d_rendu_depth_buf(0),
    _plain_phong(false),
    _textures(true),
    _draw_mesh(true),
    _raytrace(false),
    _skeleton(false),
    _rest_pose(false)
{
    _peeler    = new Peeler();
    _frame_tex = new GlTex2D(MULTISAMPX * w, MULTISAMPY * h,
                             0, GL_LINEAR, GL_CLAMP, GL_RGBA);
    allocate(w, h);
}

// -----------------------------------------------------------------------------

Render_context_cu::~Render_context_cu()
{
    _pbo_color->cuda_unregister();
    _pbo_depth->cuda_unregister();
    delete _pbo_color;
    delete _pbo_depth;
    delete _frame_tex;
    delete _peeler;

    Cuda_utils::free_d( _d_img_buffer      );
    Cuda_utils::free_d( _d_bloom_buffer    );
    Cuda_utils::free_d( _d_rendu_buf       );
    Cuda_utils::free_d( _d_rendu_depth_buf );
}

// -----------------------------------------------------------------------------

void Render_context_cu::reshape(int w, int h)
{
    allocate(w, h);
}

// -----------------------------------------------------------------------------

void Render_context_cu::allocate(int width, int height)
{
    _width  = width;
    _height = height;

    _frame_tex->bind();
    _frame_tex->set_size(MULTISAMPX * width, MULTISAMPY * height);
    _frame_tex->allocate(GL_UNSIGNED_BYTE, GL_RGBA);
    GlTex2D::unbind();

    if(_pbo_color != 0) _pbo_color->cuda_unregister();
    if(_pbo_depth != 0) _pbo_depth->cuda_unregister();

    /*
    delete _pbo_color;
    delete _pbo_depth;

    _pbo_color = new BufferObject<GL_PIXEL_UNPACK_BUFFER>(MULTISAMPX*width*MULTISAMPY*height);
    _pbo_depth = new BufferObject<GL_PIXEL_UNPACK_BUFFER>(width*height);
    */

    if( _pbo_color != 0 ) _pbo_color->set_data(MULTISAMPX*width*MULTISAMPY*height, 0);
    else                  _pbo_color = new GlBuffer_obj(MULTISAMPX*width*MULTISAMPY*height, GL_PIXEL_UNPACK_BUFFER);

    if(_pbo_depth != 0) _pbo_depth->set_data(width*height, 0);
    else                _pbo_depth = new GlBuffer_obj(width*height, GL_PIXEL_UNPACK_BUFFER);

    // Register pbos
    _pbo_color->cuda_register();
    _pbo_depth->cuda_register();

    Cuda_utils::free_d( _d_img_buffer      );
    Cuda_utils::free_d( _d_bloom_buffer    );
    Cuda_utils::free_d( _d_rendu_buf       );
    Cuda_utils::free_d( _d_rendu_depth_buf );

    Cuda_utils::malloc_d(_d_img_buffer     , width * MULTISAMPX * height * MULTISAMPY * 2 );
    Cuda_utils::malloc_d(_d_bloom_buffer   , width * MULTISAMPX * height * MULTISAMPY * 2 );
    Cuda_utils::malloc_d(_d_rendu_buf      , width * MULTISAMPX * height * MULTISAMPY     );
    Cuda_utils::malloc_d(_d_rendu_depth_buf, width * height);

    _peeler->reinit_depth_peeling(width, height);
}

// END Render_context_cu =======================================================

// -----------------------------------------------------------------------------

#include "timer.hpp"

/// Raytrace the implicit scene. and draw it with openGL onto a quad
/// @return false if the raytracing is complete.
bool raytrace(Render_context_cu* ctx, const Camera* cam)
{
    using namespace Cuda_ctrl;
    Color cl = _color.get(Color_ctrl::BACKGROUND);
    float4 cl_color = {cl.r, cl.g, cl.b, cl.a};
    const int width  = ctx->width();
    const int height = ctx->height();

    bool refresh = false;
    int*      d_img_buf = 0;
    unsigned* d_depth   = 0;
    ctx->pbo_color()->cuda_map_to( d_img_buf );
    ctx->pbo_depth()->cuda_map_to( d_depth   );
    if( ctx->_raytrace && ctx->_skeleton )
    {
        bool prog = _display._progressive_raytracing;

        if( ctx->_rest_pose ) g_skel->reset();
        refresh = !Raytracing::raytrace_implicit(*cam,
                                                 ctx->d_render_buff(), ctx->d_depth_buff(),
                                                 d_img_buf  , d_depth,
                                                 width, height, prog);
        if( ctx->_rest_pose ) g_skel->unreset();

        ctx->pbo_color()->cuda_unmap();
        ctx->pbo_depth()->cuda_unmap();

        ctx->pbo_color()->bind();
        ctx->frame_tex()->bind();
        ctx->frame_tex()->allocate(GL_UNSIGNED_BYTE, GL_RGBA);
        ctx->pbo_color()->unbind();

        EnableProgram();
        draw_quad();
        DisableProgram();
    }
    else
    {
        clean_pbos(d_img_buf, d_depth, width, height, cl_color);
        ctx->pbo_color()->cuda_unmap();
        ctx->pbo_depth()->cuda_unmap();
    }
    return refresh;
}

// -----------------------------------------------------------------------------

void draw_one_BBox(const BBox_cu &bbox){
    Point_cu pmin = bbox.pmin;
    Point_cu pmax = bbox.pmax;
    glBegin(GL_LINE_STRIP);
        glVertex3f( pmin.x, pmin.y, pmin.z );
        glVertex3f( pmin.x, pmin.y, pmax.z );
        glVertex3f( pmin.x, pmax.y, pmax.z );
        glVertex3f( pmin.x, pmax.y, pmin.z );
        glVertex3f( pmin.x, pmin.y, pmin.z );
        glVertex3f( pmax.x, pmin.y, pmin.z );
        glVertex3f( pmax.x, pmin.y, pmax.z );
        glVertex3f( pmax.x, pmax.y, pmax.z );
        glVertex3f( pmax.x, pmax.y, pmin.z );
        glVertex3f( pmax.x, pmin.y, pmin.z );
    glAssert( glEnd() );
    glBegin(GL_LINE_STRIP);
        glVertex3f( pmin.x, pmin.y, pmax.z );
        glVertex3f( pmax.x, pmin.y, pmax.z );
        glVertex3f( pmax.x, pmax.y, pmax.z );
        glVertex3f( pmin.x, pmax.y, pmax.z );
        glVertex3f( pmin.x, pmax.y, pmin.z );
        glVertex3f( pmax.x, pmax.y, pmin.z );
    glAssert( glEnd() );

}

bool display_loop(Render_context_cu* ctx, const Camera* cam)
{
    using namespace Cuda_ctrl;
    const int width  = ctx->width();
    const int height = ctx->height();
    assert(cam->width()  == width  );
    assert(cam->height() == height );

    Color cl = _color.get(Color_ctrl::BACKGROUND);
    float4 cl_color = {cl.r, cl.g, cl.b, cl.a};

    glAssert( glClearColor(cl_color.x, cl_color.y, cl_color.z, cl_color.w) );
    glAssert( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );

    ////////////////////////////////
    // Raytrace implicit surfaces //
    ////////////////////////////////
    bool refresh = false;
    Color c = Color::pseudo_rand(/*Cuda_ctrl::_debug._nb_step*/ 14);
    //g_ray_material.A  = Vec3_cu(0.3f, 0.3f, 0.3f);
    g_ray_material.A  = Vec3_cu(c.r, c.g, c.b) * 0.3f;

  //g_ray_material.Kd = Vec3_cu(255.f/255.f, 218.f/255.f, 185.f/255.f);
//    g_ray_material.Kd = Vec3_cu(0.9f, 0.83f, 0.9f);
//    g_ray_material.Kd = Vec3_cu(c.r, c.g, c.b);
    g_ray_material.Kd = Vec3_cu(1.f, 1.f, 1.f);
    g_ray_material.Ks = Vec3_cu(0.2f, 0.2f, 0.2f);
    g_ray_material.sh = 20.f;

    if (ctx->_raytrace && g_skel)
    {
        Raytracing::set_scene_type( SKELETON );
    }
#if 0
    else if (ctx->_raytrace && g_scene_blob->getNbBlobs() ){
        g_scene_blob->update();
        g_scene_blob->upload_blobs_to_gpu();
        Raytracing::set_scene_type( g_scene_blob->getOperatorType() );
    }
#endif

    refresh = raytrace(ctx, cam);

    /////////////////////////////
    // Setup projection matrix //
    /////////////////////////////

    glAssert( glMatrixMode(GL_PROJECTION) );
    glAssert( glLoadIdentity() );
    glViewport(0, 0, width, height);
    cam->gl_mult_projection();
    glAssert( glMatrixMode(GL_MODELVIEW) );
    glLoadIdentity();

    float _light0_ambient [4] = { 0.2f, 0.2f, 0.2f, 1.0f };
    float _light0_diffuse [4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    float _light0_specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    float _light0_position[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

//    glEnable(GL_LIGHTING);
//    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT , _light0_ambient );
    glLightfv(GL_LIGHT0, GL_DIFFUSE , _light0_diffuse );
    glLightfv(GL_LIGHT0, GL_SPECULAR, _light0_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, _light0_position);
    GL_CHECK_ERRORS();

    cam->lookat();


    ///////////////////////////////
    // Draw mesh with the raster //
    ///////////////////////////////
#if 1
    if(ctx->_plain_phong)
    {
        glAssert( glEnable(GL_DEPTH_TEST) );
        // Draw depth of the implicit surface (copy is slow as hell though)
        if( ctx->_raytrace )
        {
            ctx->pbo_depth()->bind();
            glAssert( glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE) );
            glAssert( glDrawPixels(width, height,GL_DEPTH_COMPONENT,GL_FLOAT,0) );
            glAssert( glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE) );
            ctx->pbo_depth()->unbind();
        }

        glColor3f(1.f, 1.f, 1.f);
        Mesh::Material mat;
        mat.setup_opengl_materials();

        if(ctx->_rest_pose)
            draw_mesh(*g_mesh, *g_animesh->get_vbo_rest_pose(), *g_animesh->get_nbo_rest_pose(), ctx->_textures);
        else
            draw_mesh(*g_mesh, g_mesh->_vbo, g_mesh->_normals_bo, ctx->_textures);

        glAssert( glDisable(GL_DEPTH_TEST) );

        if(_display._ssao) redraw_with_ssao();
    }
    else
    {
        RenderFuncWireframe rfunc(cam, ctx);

        ctx->peeler()->set_render_func(&rfunc);
        ctx->peeler()->set_background(width, height, ctx->pbo_color(), ctx->pbo_depth());
        ctx->peeler()->peel( _display._transparency );

        glAssert( glEnable(GL_DEPTH_TEST) );
        glAssert( glClear(GL_DEPTH_BUFFER_BIT) );

        // Draw depth of the depth peeling
        ctx->pbo_depth()->bind();
        glAssert( glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE) );
        glAssert( glDrawPixels(width, height,GL_DEPTH_COMPONENT,GL_FLOAT,0) );
        ctx->pbo_depth()->unbind();


        glAssert( glPolygonOffset(1.1f, 4.f) );
        glAssert( glEnable(GL_POLYGON_OFFSET_FILL) );
        rfunc.f();
        glAssert( glDisable(GL_POLYGON_OFFSET_FILL) );



        glAssert( glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE) );
        RenderFuncWireframe::render( cam, ctx );
        glAssert( glDisable(GL_DEPTH_TEST) );

    }
#endif



    ////////////////////////////////////
    // various drawing mostly sprites //
    ////////////////////////////////////
#if 1
    if(Cuda_ctrl::_debug._draw_grid_skeleton)
        Skeleton_env::draw_grids();

    if( !ctx->_plain_phong )
    {
        GLEnabledSave save_tex  (GL_TEXTURE_2D, true, false);
        GLEnabledSave save_light(GL_LIGHTING  , true, false);

        int s = Cuda_ctrl::_operators.get_display_size();
        if( _operators.get_display_controller() ) draw_controller();
        if( _operators.get_display_operator()   ) draw_operator(321, 0, s, s);
    }
#endif

    return refresh;
}

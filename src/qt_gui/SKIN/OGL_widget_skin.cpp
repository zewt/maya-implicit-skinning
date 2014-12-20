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
#include "SKIN/OGL_widget_skin.hpp"

#include <QMouseEvent>
#include <QKeyEvent>
#include <iostream>

#include "SKIN/main_window_skin.hpp"
#include "SKIN/IO_interface_skin.hpp"

#include "opengl_stuff.hpp"

// -----------------------------------------------------------------------------
#include "skeleton.hpp"
// from cuda_globals.hpp
extern Skeleton* g_skel;
// -----------------------------------------------------------------------------

void OGL_widget_skin::init_glew()
{
    glewInit();
    int state = glewIsSupported("GL_VERSION_2_0 "
                                "GL_VERSION_1_5 "
                                "GL_ARB_vertex_buffer_object "
                                "GL_ARB_pixel_buffer_object");
    if(!state) {
        fprintf(stderr, "Cannot initialize glew: required OpenGL extensions missing.");
        exit(-1);
    }
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::init()
{
    _pivot_user = Vec3_cu(0.f, 0.f, 0.f);
    _pivot = Vec3_cu(0.f, 0.f, 0.f);

    _track_pivot = false;

    _io          = new IO_interface_skin(this);

    _refresh_screen_timer = new QTimer(this);
    connect(_refresh_screen_timer, SIGNAL(timeout()), this, SLOT(updateGL()));

    initializeGL();
}

// -----------------------------------------------------------------------------

OGL_widget_skin::OGL_widget_skin(QWidget *parent, Main_window_skin* m) :
    QGLWidget(parent),
    _main_win(m)
{
    this->init();
}

// -----------------------------------------------------------------------------

OGL_widget_skin::OGL_widget_skin(QWidget *parent,  QGLWidget* sh, Main_window_skin* m):
    QGLWidget(parent, sh),
    _main_win(m)
{
    this->init();
}

// -----------------------------------------------------------------------------

OGL_widget_skin::~OGL_widget_skin()
{
    makeCurrent();
    delete _io;
    delete _refresh_screen_timer;
    delete _render_ctx;
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::initializeGL()
{
    makeCurrent();
    OGL_widget_skin::init_glew();

    // TODO: ensure this is done AFTER cuda_start()
    _render_ctx = new Render_context_cu(width(), height());
    set_draw_skeleton( false );

    _cam.set_pos(0.f, 0.f, -50.f);
    _cam.lookat(0.f, 0.f, 0.f);
    _cam.set_viewport(0, 0, width(), height());
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::resizeGL(int w, int h) {
    makeCurrent();
    glViewport(0, 0, w, h);
    _render_ctx->reshape(w, h);
    _cam.set_viewport(0, 0, w, h);
}

// -----------------------------------------------------------------------------

void test_inter(Vec3_cu p);// from testing_area.cu DEBUG

void OGL_widget_skin::paintGL()
{
    makeCurrent(); // TODO: this call might be useless check doc

    // Correct camera position if tracking is on
    update_pivot();
    update_camera();

    // Draw the scene
    bool need_to_refresh = display_loop(_render_ctx, &_cam);

    //test_inter(_cam.get_pos());//DEBUG======================================================


    if(g_save_anim || g_shooting_state)
        emit drawing();

    if( need_to_refresh )
    {
        _refresh_screen_timer->setSingleShot(true);
        _refresh_screen_timer->start(1);
    }
}

void OGL_widget_skin::set_main_window(Main_window_skin* m)
{
    _main_win = m;
}

// -----------------------------------------------------------------------------

Main_window_skin* OGL_widget_skin::get_main_window()
{
    return _main_win;
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::mousePressEvent( QMouseEvent* event ){
    makeCurrent();
    emit clicked();
    _io->mousePressEvent(event);
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::mouseReleaseEvent( QMouseEvent* event ){
    makeCurrent();
    _io->mouseReleaseEvent(event);
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::wheelEvent( QWheelEvent* event ){
    makeCurrent();
    _io->wheelEvent(event);
    event->accept();
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::mouseMoveEvent( QMouseEvent* event ){
    makeCurrent();
    _mouse_x = event->x();
    _mouse_y = event->y();

    _io->mouseMoveEvent(event);
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::keyPressEvent( QKeyEvent* event ){
    makeCurrent();
    _io->keyPressEvent(event);
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::keyReleaseEvent( QKeyEvent* event ){
    makeCurrent();
    _io->keyReleaseEvent(event);
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::enterEvent( QEvent* e){
    _is_mouse_in = true;
    e->ignore();
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::leaveEvent( QEvent* e){
    _is_mouse_in = false;
    e->ignore();
}

// -----------------------------------------------------------------------------

bool OGL_widget_skin::event(QEvent *myEvent)
{
    bool accepted = false;
    QEvent::Type type = myEvent->type();
    if(type == QEvent::KeyPress || type == QEvent::KeyRelease)
    {
        QKeyEvent* keyEvent = dynamic_cast<QKeyEvent*>(myEvent);
        int key = keyEvent->key();
        if( key ==  Qt::Key_Tab ) accepted = true;

        if(accepted){
            if(type == QEvent::KeyPress) keyPressEvent(keyEvent);
            else                         keyReleaseEvent(keyEvent);

            myEvent->accept();
            return true;
        }
    }

    return QGLWidget::event(myEvent);
}

// -----------------------------------------------------------------------------

static Vec3_cu cog_selection(bool skel_mode)
{
    using namespace Cuda_ctrl;

    // if( skel_mode )
    {
        Vec3_cu p0 = _anim_mesh->cog_mesh_selection();
        Vec3_cu p1 = _anim_mesh->cog_sample_selection();

        int s_samp = _anim_mesh->get_selected_samples().size();
        int s_mesh = _anim_mesh->get_nb_selected_points();

        if( s_samp > 0 && s_mesh > 0)
            return (p0 + p1) * 0.5f;
        else if(s_samp > 0)
            return p1;
        else
            return p0;
    }
    //    else
    //    {

    //    }
}

// -----------------------------------------------------------------------------

static Vec3_cu bone_cog(int bone_id)
{
    const Bone* b = g_skel->get_bone( bone_id );
    return (b->org() + b->end()) * 0.5f;
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::update_pivot()
{
}

// -----------------------------------------------------------------------------

void OGL_widget_skin::update_camera()
{
    if(!_track_pivot) return;

    Vec3_cu ndir = _pivot - _cam.get_pos();
    float dist   = ndir.normalize();
    Vec3_cu up   = _cam.get_y().normalized();
    Vec3_cu x    = (up.cross(ndir)).normalized();

    Vec3_cu nup = (ndir.cross(x)).normalized();

    if(_cam.get_far()*0.9 < dist)
        _cam.set_far( dist + dist*0.1);

    _cam.set_dir_and_up(ndir.x, ndir.y, ndir.z,
                        nup.x , nup.y , nup.z);
}


// -----------------------------------------------------------------------------

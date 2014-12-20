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

    // Draw the scene
    bool need_to_refresh = display_loop(_render_ctx, &_cam);

    //test_inter(_cam.get_pos());//DEBUG======================================================


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

Main_window_skin* OGL_widget_skin::get_main_window()
{
    return _main_win;
}

void OGL_widget_skin::mousePressEvent( QMouseEvent* event ){
    makeCurrent();
    emit clicked();
    _io->mousePressEvent(event);
}

void OGL_widget_skin::mouseReleaseEvent( QMouseEvent* event ){
    makeCurrent();
    _io->mouseReleaseEvent(event);
}

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

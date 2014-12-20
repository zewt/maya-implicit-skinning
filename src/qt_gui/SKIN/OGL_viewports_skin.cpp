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
#include "SKIN/OGL_viewports_skin.hpp"

#include "cuda_ctrl.hpp"
#include "macros.hpp"

void OGL_widget_skin_hidden::initializeGL(){
    OGL_widget_skin::init_glew();
    //QGLWidget::initializeGL();
    init_opengl();
}

// -----------------------------------------------------------------------------

OGL_viewports_skin::OGL_viewports_skin(QWidget* w, Main_window_skin* m) :
    QFrame(w),
    _skel_mode(false),
    _current_viewport(0),
    _main_layout(0),
    _main_window(m),
    _frame_count(0)
{

    // Initialize opengl shared context
    _hidden = new OGL_widget_skin_hidden(this);
    // Needed to initialize the opengl shared context
    _hidden->updateGL();
    _hidden->makeCurrent();

    OGL_widget_skin::init_glew();

    _hidden->setGeometry(0,0,0,0);
    _hidden->hide();
    assert( _hidden->isValid() );


    // Initialize cuda context
    std::vector<Blending_env::Op_t> op;
//    op.push_back( Blending_env::B_TCH );
    op.push_back( Blending_env::B_D  );
    op.push_back( Blending_env::U_OH );
    op.push_back( Blending_env::C_D  );

//    for (int i=(int)Blending_env::BINARY_3D_OPERATOR_BEGIN+1; i<(int)Blending_env::BINARY_3D_OPERATOR_END; ++i)
//        op.push_back( Blending_env::Op_t(i) );

    Cuda_ctrl::cuda_start( op );
    Cuda_ctrl::init_opengl_cuda();

    set_viewports_layout(SINGLE);
    QObject::connect(this, SIGNAL(active_viewport_changed(int)),
                     m   , SLOT(active_viewport(int)) );

    QObject::connect(this        , SIGNAL(update_status(QString)),
                     m->statusbar, SLOT(showMessage(QString)) );
}

// -----------------------------------------------------------------------------

OGL_viewports_skin::~OGL_viewports_skin(){
}

// -----------------------------------------------------------------------------

/////////////////DEBUG
/// The current skeleton


void OGL_viewports_skin::updateGL()
{
    using namespace Cuda_ctrl;
    _viewports[0]->makeCurrent();

    if(_skel_mode)
    {
        if(_anim_mesh != 0)
        {
            // Transform HRBF samples for display and selection
            _anim_mesh->transform_samples( /*_skeleton.get_selection_set() */);
            // Animate the mesh :
            _anim_mesh->deform_mesh();
        }
    }

    for(unsigned i = 0; i < _viewports.size(); i++)
    {
        OGL_widget_skin* v = _viewports[i];
        v->updateGL();
    }
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::enterEvent( QEvent* e){
    if(_current_viewport != 0)
        _current_viewport->setFocus();
    e->ignore();
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_viewports_layout(Layout_e setting)
{
    erase_viewports();

    _main_layout = gen_single ();

    this->setLayout(_main_layout);
    first_viewport_as_active();
}

// -----------------------------------------------------------------------------

QLayout* OGL_viewports_skin::gen_single()
{
    QVBoxLayout* vlayout = new QVBoxLayout(this);
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0, 0, 0, 0);

    {
        Viewport_frame_skin* frame = new_viewport_frame(this, 0);
        _viewports_frame.push_back(frame);
        QVBoxLayout* layout = new QVBoxLayout(frame);
        layout->setSpacing(0);
        layout->setContentsMargins(0, 0, 0, 0);
        frame->setLayout(layout);

        OGL_widget_skin* ogl = new_viewport(frame);
        _viewports.push_back(ogl);
        layout->addWidget(ogl);

        vlayout->addWidget(frame);
    }

    return vlayout;
}

void OGL_viewports_skin::erase_viewports()
{
    for(unsigned i = 0; i < _viewports.size(); i++)
    {
        _viewports[i]->close();
        delete _viewports[i];
    }

    // We don't need to delete the frames because qt will do it when deleting
    // the main layout
    _viewports_frame.clear();
    _viewports.clear();
    delete _main_layout;
    _main_layout = 0;
}

// -----------------------------------------------------------------------------

OGL_widget_skin* OGL_viewports_skin::new_viewport(Viewport_frame_skin* ogl_frame)
{
    OGL_widget_skin* ogl = new OGL_widget_skin(ogl_frame, _hidden, _main_window);
    QObject::connect(ogl, SIGNAL(drawing()), this, SLOT(incr_frame_count()));
    QObject::connect(ogl, SIGNAL( clicked() ), ogl_frame, SLOT( activate() ));
    ogl->setMinimumSize(4, 4);
    // initialize openGL and paint widget :
    ogl->updateGL();
    return ogl;
}

// -----------------------------------------------------------------------------

Viewport_frame_skin* OGL_viewports_skin::new_viewport_frame(QWidget* parent, int id)
{
    Viewport_frame_skin* frame = new Viewport_frame_skin(parent, id);
    QObject::connect(frame, SIGNAL( active(int) ),
                     this , SLOT  ( active_viewport_slot(int)) );

    return frame;
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_frame_border_color(Viewport_frame_skin* f, int r, int g, int b)
{
    QString str = "color: rgb("+
            QString::number(r)+", "+
            QString::number(g)+", "+
            QString::number(b)+");";

    f->setStyleSheet( str );
}

// -----------------------------------------------------------------------------

Vec_viewports& OGL_viewports_skin::get_viewports()
{
    return _viewports;
}

void OGL_viewports_skin::incr_frame_count()
{
    _frame_count++;
    emit frame_count_changed(_frame_count);
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::active_viewport_slot(int id)
{
    for(unsigned i = 0; i < _viewports_frame.size(); i++)
        set_frame_border_color(_viewports_frame[i], 0, 0, 0);

    set_frame_border_color(_viewports_frame[id], 255, 0, 0);
    _current_viewport = _viewports[id];

    _current_viewport->setFocus();
    _current_viewport->makeCurrent();
    emit active_viewport_changed(id);
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::first_viewport_as_active()
{
    assert(_viewports.size() > 0);
    _current_viewport = _viewports[0];
    set_frame_border_color(_viewports_frame[0], 255, 0, 0);
    _current_viewport->setFocus();
    _current_viewport->makeCurrent();
}

// -----------------------------------------------------------------------------

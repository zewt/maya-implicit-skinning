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

static void init_glew()
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

void OGL_widget_skin_hidden::initializeGL(){
    init_glew();

    //QGLWidget::initializeGL();
    init_opengl();
}

// -----------------------------------------------------------------------------

OGL_viewports_skin::OGL_viewports_skin(QWidget* w, Main_window_skin* m) :
    QFrame(w),
    _skel_mode(false),
    _main_layout(0),
    _main_window(m),
    _frame_count(0)
{

    // Initialize opengl shared context
    _hidden = new OGL_widget_skin_hidden(this);
    // Needed to initialize the opengl shared context
    _hidden->updateGL();
    _hidden->makeCurrent();

    init_glew();

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
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::enterEvent( QEvent* e){
    e->ignore();
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_viewports_layout(Layout_e setting)
{
    _main_layout = gen_single ();

    this->setLayout(_main_layout);
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

        vlayout->addWidget(frame);
    }

    return vlayout;
}

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
}

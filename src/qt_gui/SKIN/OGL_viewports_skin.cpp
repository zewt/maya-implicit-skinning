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

void OGL_widget_skin_hidden::initializeGL()
{
    init_glew();
    init_opengl();
}

// -----------------------------------------------------------------------------

OGL_viewports_skin::OGL_viewports_skin(QWidget* w, Main_window_skin* m) :
    QFrame(w),
    _skel_mode(false)
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
}

OGL_viewports_skin::~OGL_viewports_skin(){
}

void OGL_viewports_skin::updateGL()
{
    using namespace Cuda_ctrl;

    if(!_skel_mode)
        return;
    if(_anim_mesh == NULL)
        return;
    // Transform HRBF samples for display and selection
    _anim_mesh->transform_samples();
    // Animate the mesh :
    _anim_mesh->deform_mesh();
}

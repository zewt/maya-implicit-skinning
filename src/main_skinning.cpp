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
#include "SKIN/main_window_skin.hpp"
//#include "SKIN_V2/main_window_skin2.hpp"

/**
 * @file main_csg.cpp
 *
 * Initialize main window dedicated to Implicit Skinning.
 *
*/
#include <math.h>
int main(int argc, char *argv[])
{
    // Set up and show widgets.
    Main_window_skin main_window;

/*
    OGL_widget_skin_hidden* _hidden = new OGL_widget_skin_hidden(0);
    // Needed to initialize the opengl shared context
    _hidden->updateGL();
    _hidden->makeCurrent();
//    _hidden->setGeometry(0,0,0,0);
//    _hidden->hide();
    assert( _hidden->isValid() );

    int device_id = Cuda_utils::get_max_gflops_device_id();
    CUDA_SAFE_CALL( cudaDeviceReset() );
    CUDA_SAFE_CALL(cudaGLSetGLDevice(device_id) );
*/


/*
    QGLWidget* _hidden = new QGLWidget();
    // Needed to initialize the opengl shared context
    _hidden->updateGL();
    _hidden->resize(1, 1);
    _hidden->makeCurrent();
    _hidden->setGeometry(0,0,0,0);
    _hidden->hide();
    //OGL_widget_skin::init_glew();
    //QGLWidget::initializeGL();
    //init_opengl();
    assert( _hidden->isValid() );

    int device_id = Cuda_utils::get_max_gflops_device_id();
    CUDA_SAFE_CALL( cudaDeviceReset() );
    CUDA_SAFE_CALL(cudaGLSetGLDevice(device_id) );
    */

    return 0;
}

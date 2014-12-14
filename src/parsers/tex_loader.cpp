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
#include "tex_loader.hpp"

#include <QImage>
#include <QtOpenGL/QGLWidget>
#include <QFileInfo>
#include <iostream>

// =============================================================================
namespace Tex_loader{
// =============================================================================

GlTex2D* load(const std::string& file_path)
{
    QFileInfo qt_file(QString(file_path.c_str()));
    if( !qt_file.exists() || qt_file.isDir() )
    {
        std::cerr << "WARNING: can't load this texture. The file: ";
        std::cerr << file_path << " does not exists\n";

        return 0;
    }


    QImage img(file_path.c_str());

    if(img.isNull()){
        std::cerr << "WARNING: file type isn't supported for reading. ";
        std::cerr << "Can't load this texture: " << file_path << "\n";
        return 0;
    }

    QImage gl_img = QGLWidget::convertToGLFormat( img );

    GlTex2D* tex = new GlTex2D(gl_img.width(), gl_img.height(), 0,
                               GL_LINEAR_MIPMAP_LINEAR, GL_REPEAT,
                               GL_RGBA);

    GLTextureBinding2DSave save_tex_binding;
    tex->bind();
    tex->allocate(GL_UNSIGNED_BYTE, GL_RGBA, gl_img.bits());

    return tex;
}


}
// END TEX_LOADER NAMESPACE ====================================================

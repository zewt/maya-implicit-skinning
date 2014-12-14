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
#include "glshoot.hpp"

// -----------------------------------------------------------------------------

#include "port_glew.h"
#include "glassert.h"

// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <algorithm>

// When defined enable the use of Qt framework
// if not defined write ppm images in C
#define USE_QIMAGE
#ifdef USE_QIMAGE
    #include <QImage>
    #include <QDir>
    #include <QFile>
#endif

// -----------------------------------------------------------------------------

void GlShoot::gl_shoot ()
{
    glAssert( glReadBuffer (GL_FRONT) );
    glAssert( glPixelStorei(GL_PACK_ALIGNMENT, 1) );
    glAssert( glReadPixels (0, 0, _size_x, _size_y, GL_RGB, GL_UNSIGNED_BYTE, _buffer) );

    _frame_number++;
}

// -----------------------------------------------------------------------------

char* GlShoot::get_file_name(const std::string& extension) const
{
    //base_name + frame_number + "." + extension + '\0'
    const char* extension_ = extension.c_str();
    const int base_name_len = strlen (_base_name);
    const int number_len = floorf(log10f (std::max (1.f,float(_frame_number - 1))))+ 1;
    const int number_len_tot = 4;
    const int extension_len = strlen (extension_);
    const int len = base_name_len + number_len_tot + 1 + extension_len + 1;

    char * res = (char *)malloc (len * sizeof(char));
    memcpy (res, _base_name, base_name_len * sizeof (char));
    for(int i = base_name_len; i < base_name_len+number_len_tot;i++){
        res[i] = '0';
    }
    sprintf (res + base_name_len+number_len_tot-number_len, "%d", _frame_number - 1);
    res[base_name_len + number_len_tot] = '.';
    memcpy (res + base_name_len + number_len_tot + 1, extension_, extension_len * sizeof (char));
    res[len - 1] = '\0';
    return res;
}

// -----------------------------------------------------------------------------

GlShoot::GlShoot(int size_x,
                 int size_y,
                 const std::string& dir_name,
                 const std::string& file_name) :
    _size_x(size_x),
    _size_y(size_y),
    _frame_number(0)
{
    assert(_size_x >= 0);
    assert(_size_y >= 0);
    //RGB, unsigned byte
    const char* dir_name_ = dir_name.c_str();
    const char* file_name_ = file_name.c_str();
    _buffer = (uint8_t *)malloc (_size_x * _size_y * 3 * sizeof (uint8_t));

    if (dir_name_ != NULL) {
        const int dir_name_len  = strlen (dir_name_);
        const int file_name_len = strlen (file_name_);

        //base_name = dir_name_ + '/' + file_name_ + '_' + '\0'
        _base_name = (char *)malloc (dir_name_len + 1 + file_name_len + 2);
        memcpy (_base_name, dir_name_, dir_name_len * sizeof(char));
        _base_name[dir_name_len] = '/';
        memcpy (_base_name + dir_name_len + 1, file_name_, file_name_len * sizeof(char));
        _base_name[dir_name_len + 1 + file_name_len] = '_';
        _base_name[dir_name_len + 1 + file_name_len + 1] = '\0';
    } else {
        //base_name = dir_name_ + '/' + file_name_ + '_' + '\0'
        const int file_name_len = strlen (file_name_);
        _base_name = (char *)malloc (file_name_len + 2);
        memcpy (_base_name, file_name_, file_name_len * sizeof(char));
        _base_name[file_name_len] = '_';
        _base_name[file_name_len + 1] = '\0';
    }
}

// -----------------------------------------------------------------------------

void GlShoot::set_img_size(int size_x_, int size_y_)
{
    _size_x = size_x_;
    _size_y = size_y_;
    free (_buffer);
    _buffer = (uint8_t *)malloc (_size_x * _size_y * 3 * sizeof (uint8_t));
}

// -----------------------------------------------------------------------------

GlShoot::~GlShoot()
{
    free (_buffer);
    free (_base_name);
}

// -----------------------------------------------------------------------------

void GlShoot::shoot()
{
    assert (_size_x >= 0);
    assert (_size_y >= 0);

    gl_shoot();
    int width =  _size_x;
    int height = _size_y;

#ifndef USE_QIMAGE
    char* file_name = get_file_name ("ppm");
    FILE * f = fopen (file_name, "w");
    if (f == NULL) {
        fprintf (stderr, "error while opening %s for writing...\n", file_name);
        return;
    } else {
        printf ("writing %s\n", file_name);
    }
    free (file_name);
    fprintf(f,"P6\n%i %i\n255""\n",width,height);
#else
    QImage img(width, height, QImage::Format_RGB888);
    //printf ("w'%d' h'%d'\n", width, height);
    //printf ("Format '%d'\n", img.format());
#endif
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            //invert the representation, the buffer is upside down.
            int i = ((height-1) - y) * width + x;
#ifndef USE_QIMAGE
            fprintf(f, "%c%c%c", (char)m_buffer[3*i],(char)m_buffer[3*i + 1],(char)m_buffer[3*i + 2]);
#else
            img.setPixel(x,y, qRgb((char)_buffer[3*i],
                                   (char)_buffer[3*i + 1],
                                   (char)_buffer[3*i + 2]));
#endif
        }
    }
#ifdef USE_QIMAGE

    QString path_name = QString(get_file_name("bmp"));
    if( !img.save( path_name ) )
    {
        printf ("WARNING: Was not able to write '%s'\n",  path_name.toStdString().c_str());
        fflush(stdout);
    }
#else
    fclose(f);
#endif
}

// -----------------------------------------------------------------------------

int GlShoot::get_frame_nb(){
    return _frame_number;
}

// -----------------------------------------------------------------------------

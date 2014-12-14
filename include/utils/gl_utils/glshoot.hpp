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
#ifndef OPENGL_SCREENSHOOTER_HPP__
#define OPENGL_SCREENSHOOTER_HPP__


#include <string>
#include <stdint.h>

/**
 * @class GlShoot
 * @brief Taking screenshots of the current openGl window
 *
 */
struct GlShoot {

  GlShoot(int size_x,
          int size_y,
          const std::string& dir_name,
          const std::string& file_name);

  ~GlShoot();

  /// Take screenshot
  void shoot();

  int get_frame_nb();

  void set_img_size(int size_x_, int size_y_);

private:

  void gl_shoot();
  char* get_file_name(const std::string& extension_) const;

  int      _size_x;
  int      _size_y;
  uint8_t* _buffer; ///< RGB data, on 1 byte each
  char*    _base_name;
  int      _frame_number;
};

#endif // OPENGL_SCREENSHOOTER_HPP__

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
#ifndef PPM_LOADER_HPP__
#define PPM_LOADER_HPP__

#include <string>

/**
  @namespace Ppm_loader
  @brief Reading/writting '.ppm' images

  .PPM is an easy to read and write file format. It's an ASCII format.
  First line stores the width/height of the image the RGB channels are listed
  for every pixels.

  @note Due to it's ASCII nature .PPM is an highly unefficient format for both
  memory and speed performances.
*/
// =============================================================================
namespace Ppm_loader {
// =============================================================================

/// Quick and dirty ppm loader
/// writes dimensions in 'width' & 'height'
/// writes image data in 'data'
bool read(const std::string& path_name, int& width, int& height, int*& data);

bool read_with_alpha(const std::string& path_name, int& width, int& height, int*& data);


}// END PPM LOADER =============================================================

#endif // PPM_LOADER_HPP__

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
#ifndef AUTO_RIG_HPP__
#define AUTO_RIG_HPP__

#include <vector>
#include "vec3_cu.hpp"

/**
  Automatic SSD weight computation from :
  @code
  @inproceedings{Baran:2007:ARA:1275808.1276467,
    author = {Baran, Ilya and Popovi\'{c}, Jovan},
    title = {Automatic rigging and animation of 3D characters},
    booktitle = {ACM SIGGRAPH 2007 papers},
    series = {SIGGRAPH '07},
    year = {2007},
    location = {San Diego, California},
    articleno = {72},
    url = {http://doi.acm.org/10.1145/1275808.1276467},
    doi = {http://doi.acm.org/10.1145/1275808.1276467},
    acmid = {1276467},
    publisher = {ACM},
    address = {New York, NY, USA},
    keywords = {animation, deformations, geometric modeling},
  }
  @endcode



*/
void rig(const std::vector< Vec3_cu             >& vertices,
         const std::vector< std::vector<int>    >& edges,
         const std::vector< std::vector<double> >& boneDists,
         const std::vector< std::vector<bool>   >& boneVis,
         std::vector<std::vector<std::pair<int, double> > >& nzweights,
         double heat,
         int* nearest_bone,
         const std::vector<double>& nearest_bone_dist = std::vector<double>() );


#endif // AUTO_RIG_HPP__

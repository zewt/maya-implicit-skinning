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
#ifndef POINT_CACHE_EXPORT_HPP__
#define POINT_CACHE_EXPORT_HPP__

#include <vector>
#include <string>

class Point_cache_file{
public:


    Point_cache_file(int nb_points, int nb_frame_hint = 0);

    /// Add a new frame given the list of points coordinates points.
    /// @param points array of points with x y z coordinates contigus
    /// @param offset The number of elements to ignore and begin to read the
    /// array 'points'
    /// @param stride number of elements to ignore between each points
    void add_frame(float* points, int offset=0, int stride=0);

    /// Write out a mdd file
    void export_mdd(const std::string& path_name);


    // TODO:
    //void export_pc2(const std::string& path_name);
    //void import_pc2(const std::string& path_name,
    //                const std::vector< std::vector<float> >& frames);



private:
    /// list of frame. Each frame stores the object vertex coordinates.
    /// X Y Z coordinates are contigus.
    /// _frames[num_frame][list_points]
    std::vector< std::vector<float> > _frames;

    /// Number of points the cached object is.
    int _nb_points;

};

#endif // POINT_CACHE_EXPORT_HPP__

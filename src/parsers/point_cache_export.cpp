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
#include "point_cache_export.hpp"

#include <iostream>
#include <fstream>
#include <cassert>

#include "endianess.hpp"

// -----------------------------------------------------------------------------

Point_cache_file::Point_cache_file(int nb_points, int nb_frame_hint)
{
    assert(nb_points > 0);
    if(nb_frame_hint > 0)
    {
        _nb_points = nb_points;
        _frames.reserve(nb_frame_hint);
//        for(int i = 0; i < nb_frame_hint; i++)
//            _frames[i].resize(nb_points);
    }
}

// -----------------------------------------------------------------------------

void Point_cache_file::add_frame(float* points, int offset, int stride)
{
    _frames.push_back(std::vector<float>());
    const int frame_idx = _frames.size()-1;
    _frames[frame_idx].resize(_nb_points*3);
    for(int i = offset, j = 0; i < (_nb_points + offset); i++, j++)
    {
        _frames[frame_idx][j*3  ] = points[i*(3+stride)  ];
        _frames[frame_idx][j*3+1] = points[i*(3+stride)+1];
        _frames[frame_idx][j*3+2] = points[i*(3+stride)+2];
    }
}

// -----------------------------------------------------------------------------

/*
MDD file format

The first thing to note is that the MDD file format is Motorola Big Endian byte
order as opposed to the Intel Little Endian standard.
So however you implement the structure below, you must come up with an algorithm
to flip the bytes during file IO.

The data structure is like so:
typedef Struct{
    int totalframes;
    int totalPoints;
    float *Times; //time for each frame
    float **points[3];
}mddstruct;


and the data is written like so:


totalframes
totalPoints
Times
while(!totalframes)
{
    while(!totalPoints)
    {
        write point[frame][point][axis];
        point++;
    }
    frame++;
}
*/
void Point_cache_file::export_mdd(const std::string& path_name)
{
    std::ofstream file(path_name.c_str(), std::ios::binary|std::ios::trunc);

    if(!file.is_open())
    {
        std::cout << "ERROR: can't create/open " << path_name << std::endl;
        return;
    }

    int ibuff = Endianess::big_long( (int)_frames.size() );
    file.write((char*)&ibuff, 4);
    ibuff = Endianess::big_long( (int)(_nb_points)  );
    file.write((char*)&ibuff, 4);

    float fbuff[3] = {Endianess::big_float(0.1f), 0.f, 0.f};
    for(unsigned f = 0; f < _frames.size(); f++)
        file.write((char*)fbuff, 4);

    for(unsigned f = 0; f < _frames.size(); f++)
    {
        for(int p = 0; p < _nb_points; p++)
        {
            fbuff[0] = Endianess::big_float( _frames[f][p*3    ] );
            fbuff[1] = Endianess::big_float( _frames[f][p*3 + 1] );
            fbuff[2] = Endianess::big_float( _frames[f][p*3 + 2] );
            file.write((char*)fbuff, 4*3);
        }
    }
    file.close();
}

// -----------------------------------------------------------------------------

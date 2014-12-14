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
#ifndef CHRONO_H
#define CHRONO_H

#include <string>

/** @class Chrono
    @brief tool class for measuring exact process consumption time (unix rusage)
    The measured time correspond only to the time spend inside the process
*/
class Chrono
{
public:
    Chrono();

    /// start or restart chrono
    void start();

    /// Print elapsed time with a message
    void elapsedTime(std::string msge);

    double _start_time;
    double _end_time;
    double _delay;
};

#endif // CHRONO_H

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
#pragma once
#include <boost/timer.hpp>

typedef struct timeval tval_t;
/** @class Timer
    @brief boost timer wrapper class
*/
struct Timer{
    Timer() {}
    /// Restart the timer without erasing previous measured time
    /// (accessible with get_value())
    void start();
    /// stop the timer and return time in seconds
    double stop();
    /// Get last measured in seconds since the last stop() call
    double get_value();
    /// restart the timer and erase the previous results
    void reset();
private:
    boost::timer _boost_timer;
    double       _elapsed_time;
};

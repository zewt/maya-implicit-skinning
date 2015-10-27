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

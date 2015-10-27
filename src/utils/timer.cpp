#include "timer.hpp"

void Timer::start() {
    _boost_timer.restart();
}

double Timer::stop() {
    _elapsed_time = _boost_timer.elapsed();
    return _elapsed_time;
}

void Timer::reset() {
    _elapsed_time = 0.;
    _boost_timer.restart();
}

double Timer::get_value(){
    return _elapsed_time;
}

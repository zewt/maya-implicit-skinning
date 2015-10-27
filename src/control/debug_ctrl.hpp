#ifndef DEBUG_CTRL_HPP__
#define DEBUG_CTRL_HPP__

#include <vector>

class Debug_ctrl {
public:

    Debug_ctrl() :
        _potential_pit(true),
        _step_length(0.05f),
        _raphson(false),
        _collision_threshold(0.9f),
        _smooth1_iter(7),
        _smooth2_iter(1),
        _smooth1_force(1.f),
        _smooth2_force(0.5f),
        _slope_smooth_weight(2)
    {
    }

    bool _potential_pit;
    float _step_length;
    bool  _raphson;
    float _collision_threshold;

    int _smooth1_iter;
    int _smooth2_iter;
    float _smooth1_force;
    float _smooth2_force;
    int _slope_smooth_weight;
};

#endif // DEBUG_CTRL_HPP__

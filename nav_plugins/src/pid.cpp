#include "pid_velocity.hpp"

Pid::Pid(double _p,double _i,double _d){
        Kp = _p;
        Ki = _i;
        Kd = _d;
        // integral = 0;
}


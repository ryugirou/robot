#include "pid_position.hpp"

double Pid::update(const double& sensor_val,const double& target_val,const double& dt){
        diff[0] = diff[1];
        diff[1] = target_val - sensor_val;
        integral += diff[1] * dt;
        return Kp * diff[1] + Ki * integral + Kd * (diff[1]- diff[0]) / dt;
}
#pragma once

class Pid{
  private:
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double diff[2]={0,0};
  public:
    Pid(double _p,double _i,double _d):Kp(_p),Ki(_i),Kd(_d),integral(0){};
    double update(const double& sensor_val,const double& target_val,const double& dt);
};

double Pid::update(const double& sensor_val,const double& target_val,const double& dt){
        diff[0] = diff[1];
        diff[1] = target_val - sensor_val;
        integral += diff[1] * dt;
        return Kp * diff[1] + Ki * integral + Kd * (diff[1]- diff[0]) / dt;
}

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



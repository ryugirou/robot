#pragma once

class Pid{
  private:
    double Kp;
    double Ki;
    double Kd;
    double e[3]={0,0,0};
    double u[2]={0,0};

    double du;
  public:
    Pid(double _p,double _i,double _d):Kp(_p),Ki(_i),Kd(_d){};
    double update(const double& sensor_val,const double& target_val,const double& dt);
};

double Pid::update(const double& sensor_val,const double& target_val,const double& dt){
  e[0] = e[1];
  e[1] = e[2];
  e[2] = target_val - sensor_val; 

  du = Kp*(e[2]- e[1]) + Ki*e[2]*dt + Kd*(e[2]-2*e[1]+e[0])/dt;

  u[0] = u[1];
  u[1] = u[0] + du;
  return u[1];  
}
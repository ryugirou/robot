#ifndef   PID
#define   PID

class Pid{
  private:
    double Kp;
    double Ki;
    double Kd;
    double e[3]={0,0,0};
    double u[2]={0,0};

    double du;
  public:
    Pid(double _p,double _i,double _d);
    inline double update(double sensor_val,double target_val,double dt);
};

inline double Pid::update(double sensor_val,double target_val,double dt){
  e[0] = e[1];
  e[1] = e[2];
  e[2] = target_val - sensor_val; 

  du = Kp*(e[2]- e[1]) + Ki*e[2]*dt + Kd*(e[2]-2*e[1]+e[0])/dt;

  u[0] = u[1];
  u[1] = u[0] + du;
  return u[1];  
}
#endif // PID
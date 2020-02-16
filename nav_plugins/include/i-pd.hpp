#ifndef   PID
#define   PID

class Pid{
  private:
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double sensor[2]={0,0};
  public:
    Pid(double _p,double _i,double _d);
    inline double update(double sensor_val,double target_val,double dt);
};

inline double Pid::update(double sensor_val,double target_val,double dt){
        sensor[0] = sensor[1];
        sensor[1] = sensor_val;
        integral += (target_val - sensor_val) * dt;
        return Kp * sensor_val  + Ki * integral + Kd * (sensor[1]- sensor[0]) / dt;
}

#endif // PID
#ifndef   PID
#define   PID

class Pid{
  private:
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double diff[2]={0,0};
  public:
    Pid(double _p,double _i,double _d);
    inline double update(double sensor_val,double target_val,double dt);
};

inline double Pid::update(double sensor_val,double target_val,double dt){
        diff[0] = diff[1];
        diff[1] = target_val - sensor_val;
        integral += diff[1] * dt;
        return Kp * diff[1] + Ki * integral + Kd * (diff[1]- diff[0]) / dt;
}


#endif // PID
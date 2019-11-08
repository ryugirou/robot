#ifndef   PID
#define   PID

class Pid{
  private:
    double Kp;
    double Ki;
    double Kd;
    double diff[2]={0,0};
    double u[2]={0,0};
    double ud[2]={0,0};

    
    double Tf =0.0492;
    double f;
    double g;
    double du;
  public:
    Pid(double _p,double _i,double _d);
    inline double update(double error,double dt);
};

Pid::Pid(double _p,double _i,double _d){
        Kp = _p;
        Ki = _i;
        Kd = _d;
}

inline double Pid::update(double error,double dt){
  f=Tf/(Tf+dt);
  g=Kd/(Tf+dt);
    
  diff[0] = diff[1];
  diff[1] = error; 
    
  ud[0] = ud[1];
  ud[1] = f*ud[0]+g*(diff[1]-diff[0]);

  du    = Kp*(diff[1]-diff[0])+dt*Ki*diff[1]+Kd*(ud[1]-ud[0]);

  u[0] =u[1];
  u[1]  = u[0]+du;
  return u[1];  
}
#endif // PID

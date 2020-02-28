#include <array>
#include <qpOASES.hpp>
#define N 10
//モデル
struct X
{
  float x,y,yaw;
};

struct U
{
  float vx,vy,omega;
};

struct Model{
  X Predict(const X& state,const U& input,const float& dt)
  {
    return {state.x + input.vx*dt,state.y + input.vy*dt,state.yaw + input.omega*dt};
  }
};

//最適化IPOPT

//評価関数

//main
class MPC
{

}
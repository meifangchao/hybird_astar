#include "node3d.h"
#include <iostream>
using namespace HybridAStar;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements
//const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
//const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
//const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

// 扩展节点的配置表
// R = 6, 6.75 DEG

const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

// R = 3, 6.75 DEG
//const float Node3D::dy[] = { 0,        -0.0207946, 0.0207946};
//const float Node3D::dx[] = { 0.35342917352,   0.352612,  0.352612};
//const float Node3D::dt[] = { 0,         0.11780972451,   -0.11780972451};
// const float Node3D::dys[] = { 0,        -0.0415893,  0.0415893};
// const float Node3D::dxs[] = { 0.7068582,   0.705224,   0.705224};
// const float Node3D::dts[] = { 0,         0.1178097,   -0.1178097};

const float Node3D::dyf[] = { 0,       -0.16578, 0.16578};
const float Node3D::dxf[] = { 1.41372, 1.40067, 1.40067};
const float Node3D::dtf[] = { 0,       0.2356194,   -0.2356194};

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && 
  (int)(t / Constants::deltaHeadingRad) >= 0 && 
  (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}


//###################################################
//                                        IS IN RANGE
//###################################################


// 除了加上时间 还可以考虑加上扩展点数的限制
//   iter 为队列完成 pop() 的次数
// t 为时间 毫秒单位
bool Node3D::limit_isInRange(const Node3D& goal,int iter,float t) const {
  // 随即rand   1-10
  int random = rand() % 10 + 1;

  // 越靠近终点 dx dy 越小
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;

  bool rand = (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
  // 300ms 内没成功 强制触发 
  bool use_time_flag = t > 300;
  bool ite_count_flag = iter > 600;
  // 导致返回 true 的可能性变大
  //             距离的平方 小于          100 
  return rand || (use_time_flag) || ite_count_flag;
}


//
float Node3D::distance(const Node3D& another){
  float dx = std::abs(x - another.x);
  float dy = std::abs(y - another.y);
  float dis = (dx * dx) + (dy * dy);
  //
  // std::cout << "起点到终点距离的平方为: " << dis << std::endl;
  return dis;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################



// Node3D* Node3D::createSuccessor(const int i) {

//   // 新建节点的 x y t
//   float xSucc;
//   float ySucc;
//   float tSucc;

//   // 前进方向的扩展节点
//   /* 
//   const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
//   const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
//   const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};
//   */


//   // 坐标系 
//   if (i < 3) {
//     xSucc = x + dxf[i] * cos(t) - dyf[i] * sin(t);
//     ySucc = y + dxf[i] * sin(t) + dyf[i] * cos(t);
//     tSucc = Helper::normalizeHeadingRad(t + dtf[i]);
//   }
//   // 后退方向的扩展节点
//   else {
//     xSucc = x - dxf[i - 3] * cos(t) - dyf[i - 3] * sin(t);
//     ySucc = y - dxf[i - 3] * sin(t) + dyf[i - 3] * cos(t);
//     tSucc = Helper::normalizeHeadingRad(t - dtf[i - 3]);
//   }

//   // 堆空间申请节点 并返回  用父亲节点的g值 赋值给儿子
//   return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);

// }





// 自己写的 考虑局部障碍物的距离
// 行不通
Node3D* Node3D::createSuccessor_conside_obs(const int i,float dis){

    // 新建节点的 x y t
    float xSucc;
    float ySucc;
    float tSucc;
    float r = std::sqrt(Constants::length * Constants::length + Constants::width*Constants::width);
    float kk;
    // 建立一个kk 与 dis的函数 
    // 这里是简单的分段函数
    // 离障碍物较远 才加大步伐
    // TODO 这里调参的时候 数字稍微大一点 在空旷地方慢慢靠近障碍物过程中 就容易规划出离奇轨迹
    if(dis > r + 4){
      kk = 1.03;
    }
    else if(dis > r + 1 && dis < r + 4){
      kk = 1.01;
    }
    else{
      kk = 1;
    }
    

    if (i < 3) {
      xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
      ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
      tSucc = Helper::normalizeHeadingRad(t + dt[i]);
    }
    // 后退方向的扩展节点
    else {
      xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
      ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
      tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
    }

    // 堆空间申请节点 并返回  用父亲节点的g值 赋值给儿子
  
    return new Node3D(kk * xSucc, kk* ySucc, tSucc, g, 0, this, i);

}


//###################################################
//   计算移动成本
//###################################################

void Node3D::updateG() {
  // 求解距离最近障碍物的距离 

  // 该节点来自 前进方向扩展的 0 1 2 
  if (prim < 3) {
    // 如果该节点扩展方向 和 上一个是不一致的 对其进行惩罚
    if (pred->prim != prim) {
      // 来自反方向 
      if (pred->prim > 2) {
        /*
            const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
            const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
            const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

        */
        //  这里什么需要使用 dx[0] 作为基数？？
        //                   1.05                  5    
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } 
      // 来自同一大方向
      else {
        //                   1.05
        g += dx[0] * Constants::penaltyTurning;
      }
    } 
    // 扩展前后 方向不变动
    else {
      g += dx[0];
    }
  }
  // 该节点来自 后退方向扩展的
  // 很明显 相当排斥倒车

  else {
    
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        //                   1.05                         2.0                            
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing 
                    * Constants::penaltyCOD;
      } 
      else {
        //                     1.05                       2
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } 
    else {
      g += dx[0] * Constants::penaltyReversing ;
    }
  }
}

// 一般的G代价 在 10 的数量级 
// 考虑 靠近障碍物的代价的函数 如果过近代价加大一点 

void Node3D::append_obs_g(float dis){
    // 内部当前节点最近障碍物的距离 追加代价
    // 考虑车辆大小 2.65 * 1.75 可以考虑车辆的位姿
    float half_len = Constants::length / 2;
    float half_wid = Constants::width / 2;
    // 1.58m 
    float radius = std::sqrt(half_len*half_len + half_wid*half_wid);
    // 0.5m开外 都无障碍物
    if(dis > radius + 0.5){
      // g 值无需改变 

    } 
    else if(dis > half_wid && dis < radius + 0.5){
        // 设置一个简单线性函数 进行代价计算 g值需要变化
        
        float temp = (((radius + 0.5) - dis) / ((radius + 0.5) - half_wid)) * Constants::appden_max;
        g = g + temp;                      
    }
    else{
      // 强制赋值很高
        g = g + 100;
    }

}

//###################################################
//                                 3D NODE COMPARISON
//###################################################
// 判断两个节点 是否相等的重载方法设定
// 这是存在问题的 仅仅只是保证了在一个大的珊格地图内而已
bool Node3D::operator == (const Node3D& rhs) const {
  // 节点的具体为是是float类型 这里转化为int 确保两者在同一个珊格内
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         // 角度差阈值  偏差在一个角度分辨率之内
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);


  // return std::abs(x - rhs.x) <= 0.3   &&
  //        std::abs(y - rhs.y) <=  0.3  &&
  //        // 角度差阈值  偏差在一个角度分辨率之内
  //        (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
  //         std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}

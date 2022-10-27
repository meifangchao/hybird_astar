#include "collisiondetection.h"
#include<iostream>
using namespace HybridAStar;

// 构造函数
CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  // 3D碰撞查询表 初始化
  Lookup::collisionLookup(collisionLookup);
}

// 
//  3D 节点的碰撞检测 本质就是加大采样 然后遍历每一中可能
  // 碰撞检测 将地图分成 10 * 10 * 72的分辨率
// 没有碰撞返回true 存在碰撞风险 返回false
bool CollisionDetection::configurationTest(float x, float y, float t) const {

  int X = (int)x;
  int Y = (int)y;

  //  浮点向整型转换必定丢失精度                        10
  //            int{（5.56 - 5）* 10 }
  //  这一步 就是定位 该位置在10*10的单位小珊格的位置S
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  
  //                                  10
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;


  //                  t / 2* pi / 72
  int iT = (int)(t / Constants::deltaHeadingRad);
  // 这里是 求出该3D node节点在碰撞检测地图单元格的编号 （注意这里是不同小珊格内的编号）
  int idx = iY * Constants::positionResolution * Constants::headings 
            + iX * Constants::headings 
            + iT;

  int cX;
  int cY;

  // 遍历检测 只有所有的 都没有被碰撞 才认定为无碰撞
  // 当前位置 车辆所占用的珊格集合
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    
    // 当前位置 + 车辆占用的大珊格图的偏离量
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // 确保求出的位置 在地图内部 才进行碰撞检测 如果越界怎么办
    // 车辆占用珊格出界了
    if (cX >= 0 && (unsigned int)cX < grid->info.width && 
        cY >= 0 && (unsigned int)cY < grid->info.height) {
      // 查询珊格地图 判断该位置 是否碰撞
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }
    }


    // 修改逻辑
    else{
      // 
     // std::cout << "----------------------------" << std::endl;
     //  std::cout << "dubins曲线有节点越界了,越界节点坐标为：" << "(" << cX << "," << cY << ")" << std::endl; 
      return false; 
    }

  }

  return true;
}
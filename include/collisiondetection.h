#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {

// 计算2D节点的 x y theta（默认为 99） 
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

// 计算3D节点的 x y theta 
inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/

// 碰撞检测类
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();


  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */

  // 该模板函数 用于计算该节点（3D节点 / 2D节点） 是否有碰撞风险）
  // 很重要 3D节点的碰撞检测 在两个地方用到 一个是hybird_a* 扩展节点 时候用到
  //                                  一个是对dubins曲线的采样的时候用到
  //       2D节点的碰撞检测 只有在astar 扩展的时候用到了
  template<typename T> bool isTraversable(const T* node) const {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;

    // 获取该节点的  x y theta信息
    // 如果是2d节点 t == 99 恒成立
    getConfiguration(node, x, y, t);

    // 2D collision test
    // 2D 节点碰撞检测 如果该节点在珊格地图上为障碍物 直接返回碰撞
    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    // 3D 节点的碰撞检测
    if (true) {
      // 3D 节点检测碰撞的核心
      cost = configurationTest(x, y, t) ? 0 : 1;
    } 
    
    else {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;
  }



  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) const {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t) const;

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

 private:

  /// The occupancy grid
  // 珊格地图
  nav_msgs::OccupancyGrid::Ptr grid;

  /*
  // 车辆在每种角度下 占用的珊格地图数量
      struct config {
        /// the number of cells occupied by this configuration of the vehicle
        int length;
        
          \var relPos pos[64]
          \brief The maximum number of occupied cells
          \todo needs to be dynamic
        
        relPos pos[64];
      };
        
  */
  /// 车辆在每一个角度情况下占用的数量
  // 碰撞风险表                               72 * 10 * 10  每一个珊格离散成100
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H

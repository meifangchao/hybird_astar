#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>
#include<iostream>
#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
// 平滑类
class Smoother {
 public:
  Smoother() {}

  /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.
   // 该函数采用由节点组成的路径，并尝试使用梯度下降迭代地平滑该路径。
     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */

   // 这个函数很重要 优化的核心入口函数
  void smoothPath(DynamicVoronoi& voronoi);

  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
  void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());

  /// returns the path of the smoother object
  const std::vector<Node3D>& getPath() {
    // 打印出所有的路径点
    for(auto &m :path){
      std::cout <<  "x值: "<< m.getX()<< "  " << "y值: " << m.getY() << "  " << "theta值: " << m.getT() << std::endl;
    }

    std::cout << std::endl << std::endl;
    std::cout << "--------------------------------" << std::endl;
    return path;
    }

  /// obstacleCost - pushes the path away from obstacles
  Vector2D obstacleTerm(Vector2D xi);

  /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
  Vector2D curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2);

  /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
  // 尝试以相同的方向等距离地分布节点
  Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

  /// voronoiCost - trade off between path length and closeness to obstaclesg
  //   Vector2D voronoiTerm(Vector2D xi);

  /// a boolean test, whether vector is on the grid or not
  // 判断向量 是不是在地图范围内
  bool isOnGrid(Vector2D vec) {
    if (vec.getX() >= 0 && vec.getX() < width &&
        vec.getY() >= 0 && vec.getY() < height) {
      return true;
    }
    return false;
  }
 

 private:

  /// maximum possible curvature of the non-holonomic vehicle
  // 最小转弯半径 对应的最大转弯曲率      6
  float kappaMax = 1.f / (Constants::r * 1.1);

  /// maximum distance to obstacles that is penalized
  //  障碍物最小安全距离         2
  float obsDMax = Constants::minRoadWidth;

  /// maximum distance for obstacles to influence the voronoi field
  // 障碍物 在voronoi 图中的最大影响范围   2
  float vorObsDMax = Constants::minRoadWidth;

  /// falloff rate for the voronoi field
  // voronoi 场的衰减率 可以理解成迭代的学习率
  float alpha = 0.1;

   /// 各类优化权重系数
  /// weight for the obstacle term
  float wObstacle = 0.2;

  /// weight for the voronoi term
  float wVoronoi = 0;

  /// weight for the curvature term
  float wCurvature = 0.1;

  /// weight for the smoothness term
  float wSmoothness = 0.2;

  /// voronoi diagram describing the topology of the map
  // 描述地图拓扑的voronoi图 
  DynamicVoronoi voronoi;

  /// width of the map
  int width;
  /// height of the map
  int height;

  // 将用于优化的 原始path 
  std::vector<Node3D> path;
  
};
}
#endif // SMOOTHER_H

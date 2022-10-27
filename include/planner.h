#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:

  DynamicVoronoi share_vro;

  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
 // 该函数可能会大大加快搜索速度
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
 // 设置地图函数
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan();

 private:
  //  ros 的节点句柄
  /// The node handle
  ros::NodeHandle n;

  /// A publisher publishing the start position for RViz
  // 发布节点
  ros::Publisher pubStart;

  /// A subscriber for receiving map updates
  ros::Subscriber subMap;

  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;

  /// A listener that awaits transforms
  tf::TransformListener listener;
  /// A transform for moving start positions
  tf::StampedTransform transform;

  /// The path produced by the hybrid A* algorithm
  Path path;

  /// The smoother used for optimizing the path
  Smoother smoother;

  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);

  /// The visualization used for search visualization
  Visualize visualization;

  /// 碰撞检测类 
  CollisionDetection configurationSpace;

  /// voronoi 图类 用于优化
  DynamicVoronoi voronoiDiagram;

  Algorithm  tool_a;
  // static DynamicVoronoi vor_forcost;


  /// A pointer to the grid the planner runs on
  // 这表示一个二维网格图，其中每个单元格表示占用概率。
  // # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  Unknown is -1.
   // int8[] data
   // 数据为一维的形式 将二维信息 统一转换成一维
   // 栅格地图中坐标是一维数组的形式，比如实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
   //   grid->data[node->getIdx()];
   // 
  nav_msgs::OccupancyGrid::Ptr grid;

  /// The start pose set through RViz
  // 利用它 读取start的位置坐标的信息
  geometry_msgs::PoseWithCovarianceStamped start;
  
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;

  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;

  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];

  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H

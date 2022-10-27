/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"

#include "dynamicvoronoi.h"


//###################################################
//                              COUT STANDARD MESSAGE
//###################################################
/**
   \fn message(const T& msg, T1 val = T1())
   \brief Convenience method to display text
*/
// 输出消息以及其对应值的模板函数 
template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
//             包名
// roslaunch hybrid_astar manual.launch
int main(int argc, char** argv) {

  // extern HybridAStar::DynamicVoronoi voronoi_al;
  // DynamicVoronoi voronoi_al;
  message<string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } 
  else {
    message("mode: ", "auto");
  }

  // 初始化 ros 设置一系列的初始化
  //                    节点名称
  /*
      os::init()的参数分别是：
      argc：remapping参数的个数
      argv：remapping参数的列表
      name：节点名，必须是一个基本名称，不能包含命名空间
      options：[可选]用于启动节点的选项（ros::init_options中的一组位标志）

      调用ros::init()函数后，它会调用五个函数：
      network::init(remappings);
      master::init(remappings);
      this_node::init(name, remappings, options);
      file_log::init(remappings);
      param::init(remappings);

  */

  ros::init(argc, argv, "a_star");

  // 建立planner 类对象 该对象作为接口 进行hybird A*算法
  // 对象建立时（构造函数） 会初始化很多操作  比如回调函数 对起点终点的设置等
  HybridAStar::Planner hy;
  // 开始规划 内部调用hybird_a* 平滑等算法
  
  hy.plan(); 
  
  
  // 订阅者不断回调 
  ros::spin();
  return 0;
}

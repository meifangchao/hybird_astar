#include "path.h"

using namespace HybridAStar;
#include <fstream>


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}


//###################################################
//                                         TRACE PATH
//##################################################
// 根据回溯找出所有的路径信息  设置可视化的信息 
// 该函数由 planner内部调用
void Path::updatePath(const std::vector<Node3D>& nodePath) {

  path.header.stamp = ros::Time::now();
  int k = 0;
  // 遍历node 节点的 vector大小
  for (size_t i = 0; i < nodePath.size(); ++i) {

    // 依次添加每一个节点 并设置参数
    // 设置节点位姿 主题发布
    addSegment(nodePath[i]);
    // 内部marker 类 实现node节点的可视化信息设置
    addNode(nodePath[i], k);
    // 记录节点个数
    k++;
    // 车辆可视化信息设置
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D& node) {

  // 节点位姿的标准信息
  geometry_msgs::PoseStamped vertex;

  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;

  // 旋转变量 统一设为0 
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;

  path.poses.push_back(vertex);
}


// 添加节点 并设置节点的可视化参数
// 参数 i 决定清除之前的设置
void Path::addNode(const Node3D& node, int i) {

  // marker 类 实现node节点的可视化信息设置
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    // action操作，是添加还是修改还是删除
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } 
  else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}


// 设置可视化车辆的参数
void Path::addVehicle(const Node3D& node, int i) {

  // 车辆可视化标准类
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;

  // 大小     
  //                                  2.65  
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  //                              
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } 
  else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());

  pathVehicles.markers.push_back(pathVehicle);
}

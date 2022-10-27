#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/

 //  3D节点类
class Node3D {
 public:

  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments

  // 构造函数
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }

  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the heading theta
  void setT(const float& t) { this->t = t; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }

  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) { 
    //                          
    this->idx = (int)(t / Constants::deltaHeadingRad) * width * height 
                + (int)(y) * width + (int)(x); 
  return idx;}
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D* pred) { this->pred = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();

  void append_obs_g(float dis); 
  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const Node3D& rhs) const;

  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  bool isInRange(const Node3D& goal) const;


  bool limit_isInRange(const Node3D& goal,int iter,float t) const;


  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const;

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  // 在连续空间内 建立后继结点
  Node3D* createSuccessor(const int i);

  Node3D* createSuccessor_conside_obs(const int i,float dis);


  float distance(const Node3D& another);
  // 常量
  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static const float dx[];
  /// Possible movements in the y direction
  static const float dy[];
  /// Possible movements regarding heading theta
  static const float dt[];

    static const float dxf[];
  /// Possible movements in the y direction
  static const float dyf[];
  /// Possible movements regarding heading theta
  static const float dtf[];


//  成员呀
 private:

  // x 的位置值
  float x;
  // y 的位置值
  float y;

  /// the heading theta
  // 航向角 theta 
  float t;

  //  每个节点代价值 H 和 G值
  /// 到达该节点的累积的G值代价
  float g;

  // 该节点到达终点需要的预估代价
 
  float h;

  // 节点编号
  
  int idx;


  // 该节点是否在开放列表 关闭列表
  
  bool o;
  /// the closed value
  bool c;

  
  // 0 1 2 代表该节点是由 上一个节点前进得到的
  // 3 4 5 代表该节点是由 上一个节点后退得到的
  int prim;

  // 该节点的前继节点指针 
  /// the predecessor pointer
  const Node3D* pred;
};
}
#endif // NODE3D_H

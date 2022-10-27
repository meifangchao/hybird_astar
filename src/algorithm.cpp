#include "algorithm.h"
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include "planner.h"
#include<ros/time.h> 
#include <ros/duration.h>
#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

void write_time_dubins(string file_name1,ros::Duration dur);

void write_time_dubins(string file_name1,ros::Duration dur)
{
  // 1.包含头文件 fstream
  //      2.创建流对象
  ofstream ofs_xyt;
  
  ofs_xyt.open(file_name1, ios::app);
  
  ofs_xyt << dur * 1000 << endl; 

 
  ofs_xyt.close();
  
}

void write_count(string file_name1,int count);


void write_count(string file_name1,int count)
{
  // 1.包含头文件 fstream
  //      2.创建流对象
  ofstream ofs_xyt;
  
  ofs_xyt.open(file_name1, ios::app);
  
  ofs_xyt << count << endl; 

 
  ofs_xyt.close();
  
}

// astar算法入口
float aStar(Node2D &start, Node2D &goal, Node2D *nodes2D,
            int width, int height,
            CollisionDetection &configurationSpace,
            Visualize &visualization);

void updateH(Node3D &start, const Node3D &goal, Node2D *nodes2D, float *dubinsLookup,
             int width, int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization);

// dubins曲线的生成
Node3D *dubinsShot(Node3D &start,
                   const Node3D &goal,
                   CollisionDetection &configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/

// 节点比较方法的重载  由小到大
struct CompareNodes
{
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  // 3D节点
  bool operator()(const Node3D *lhs, const Node3D *rhs) const
  {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  // 2D节点
  bool operator()(const Node2D *lhs, const Node2D *rhs) const
  {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
// hybird A* 算法入口
// 返回node3 也就是算法扩展的最后一个节点 结果指针序列
// 回过头看 planner调用了这个核心函数
Node3D *Algorithm::hybridAStar(Node3D &start,      // 起始点
                               const Node3D &goal, // 终止点
                               // 3D 节点的指针容器
                               // Node3D* nodes3D = new Node3D[length]();
                               // 2D 节点的指针容器
                               // Node2D* nodes2D = new Node2D[width * height]();
                               Node3D *nodes3D,
                               Node2D *nodes2D,
                               int width, // 地图大小参数 分辨率
                               int height,
                               CollisionDetection &configurationSpace, // 空间障碍物占用表
                               float *dubinsLookup,
                               // 可视化类
                               Visualize &visualization,
                               Algorithm & al
                               )
{

  ros::Time start_a = ros::Time::now();

  bool first_run = true;


  int dubins_count = 0;
  // 计算起点与终点直接的距离
  bool distance_flag = false;
  // 该阈值 最好设置成和 转弯半径相关  距离很近的时候 可以
  if (start.distance(goal) < 36)
  {
    distance_flag = true;
  }

  // PREDECESSOR AND SUCCESSOR INDEX
  // 父节点 和 后继节点
  int iPred, iSucc;

  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  // dir = 6 代表可以掉头搜索 dir = 3 代表只能前进搜索
  // 默认开启该后退开关
  int dir = Constants::reverse ? 6 : 3;
  // 当前迭代次数
  int iterations = 0;

  // VISUALIZATION DELAY
  // 可视化类延时时间
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  // open集合优先队列
  // 二项堆是是二项树的集合或是由一组二项树组成。二项堆具有良好的性质。在
  // 的时间内即可完成两个二项堆合并操作，所以二项堆是可合并堆，而仅仅需要的时间，二项堆即可完成插入操作。
  // 因此，基于二项堆实现的优先队列和进程调度算法有着很好的时间性能。
  // 同时由于二项树的结构特性和性质，二项树在网络优化等诸多领域也应用广泛
  typedef boost::heap::binomial_heap<Node3D *,
                                     boost::heap::compare<CompareNodes>>
      priorityQueue;

  // open 队列
  priorityQueue O;

  // 首先就计算 起点到终点的预估代价

  updateH(start, goal,
          nodes2D, dubinsLookup,
          width, height,
          configurationSpace,
          visualization);

  // mark start as open
  // void close() { c = true; o = false; }
  // 设置初始节点在 close集合内
  start.open();

  // push on priority queue aka open list
  // 将初始节点 加入open集合中 属于初始化操作
  O.push(&start);

  // 求解起始点的3D节点编号
  iPred = start.setIdx(width, height);

  // 在node3d 的堆空间为其赋值
  nodes3D[iPred] = start;

  // NODE POINTER
  // 父亲节点 儿子节点
  Node3D *nPred;
  Node3D *nSucc;

  // float max = 0.f;

  // continue until O empty
  // 类似 A* 进行优先队列范围内搜索 直到找到结果 或者没有结果就退出


  while (!O.empty())
  {
    ros::Time suanfa_s = ros::Time::now();
    
    // pop node with lowest cost from priority queue
    // 取出open集合中 代价最小的节点
    nPred = O.top();
    // 计算并设置该 3D node的编号
    // set index
    iPred = nPred->setIdx(width, height);

    // 记录运算过的节点次数+1 超过阈值 判定算法失败
    
    iterations++;

   

    // RViz visualization
    // 3D节点可视化开关 默认关闭
    if (Constants::visualization)
    {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    // 如果该节点 已经在close集合内 直接舍弃该节点 不进行扩展 从open 中重新计算下一个
    if (nodes3D[iPred].isClosed())
    {

      O.pop();
      continue;
    }
    // _________________
    //  该节点 不在close 集合内 扩展该节点的后继
    else if (nodes3D[iPred].isOpen())
    {

      // 加入到close 集合内 并从open中移除该节点
      nodes3D[iPred].close();
      // 将该节点的数据 H G total total_cost

      // close 集合移除该节点
      O.pop();

      // 判断是否已经找到目标 或者已经迭代超时          30000
      // 这里超出迭代次数 会导致显示出来的轨迹不完整（仅仅是搜索过的一部分）
      if (*nPred == goal || iterations > Constants::iterations)
      {

        cout << "A*算法完成规划时，剩余的队列的容量: " <<  O.size() << endl;
        // 输出迭代次数
        if(iterations > Constants::iterations){
          cout << "****************************" << endl;
          cout << "迭代次数过多,并且迭代次数为:" << iterations << endl;
          cout << "hybird_a*最多可迭代次数为:" << Constants::iterations << endl;

        }
        else{
          cout << "****************************" << endl;
          cout << "总共触发了dubins " << dubins_count << " 次" << std::endl;
          cout << "最终由A*扩展规划成功，生成了可执行路径"  << endl;
          cout << "迭代次数为:" << iterations << endl;

        }

        write_count("each_dubins_count.txt",dubins_count);
        write_count("each_iter_count.txt",iterations);
        // 返回结果 注意 不一定是目标的终点节点 有可能是超时扩展的最后一个节点
        return nPred;
      }

      // 没有到达终点 继续扩展
      else
      {
        // 第一次运行就采用dubins 

        // 进行dubins检查 看看是否可以直接通过dubins获取结果
        // dubins开关打开 dubinsshot是不允许倒车  并且车辆到达终点附近
        ros::Time now_1 = ros::Time::now();

        ros::Duration a_dur(now_1 - start_a);
        // 距离算法开始的时间
        float used_time = a_dur.toSec()*1000;
        
        // float now_time = (float)(a_dur*1000);

        //  cout << "used_time " << used_time << endl;
        if ((Constants::dubinsShot &&

            // 内部进行随即random 靠近终点 isInRange得到true的可能性变大
            // nPred->isInRange(goal) &&
            nPred->limit_isInRange(goal,iterations,used_time) &&
            //   并且该节点是由前进扩展来的
            nPred->getPrim() < 3 &&
            // 终点与起点距离比较近 没必要进行dubins
            !distance_flag ) 
            || first_run
            )
        {
          // 搞出每一次 dubins需要的时间
          ros::Time dubins_s = ros::Time::now();
          
          // 第一次届促发后 就关闭该开关

          first_run = false;
          dubins_count++;
          // cout << "触发了dubins " << dubins_count << " 次" << std::endl;

          // 进行dubins算法 看看能不能直接到达终点
          // 内部加入了判定长度的逻辑  如果长度过长 可以不采纳dubins曲线的结果
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          ros::Time dubins_end = ros::Time::now();
          ros::Duration dur_dubins(dubins_end - dubins_s);
          write_time_dubins("dubins_time_cost.txt",dur_dubins);

          // dubins 算法成功 直接算法结束 返回结果
          if (nSucc != nullptr && *nSucc == goal)
          {
            // DEBUG
            //  std::cout << "max diff " << max << std::endl;

            // 输出触发了dubins曲线
            cout << "dubins完成规划时,剩余的队列的容量: " <<  O.size() << endl;

            cout << "由于触发了dubins而成功最后规划" << std::endl;
            cout << "触发了dubins而成功时: x y 坐标为" << nPred->getX() << "  " <<  nPred->getY() << endl;
            cout << "总共触发了dubins " << dubins_count << " 次" << std::endl;
            cout << "迭代次数为:" << iterations << endl;
            write_count("each_dubins_count.txt",dubins_count);
            write_count("each_iter_count.txt",iterations);
            return nSucc;
          }
        }

        // ______________________________
        // dubins暂时无结果 或者没有触发dubins 使用前向的模拟扩展
        //                  6
        for (int i = 0; i < dir; i++)
        {
          // 扩展算法的核心  非常重要
          // 创建可能的候选节点 根据编号i 进行扩展方向以及控制量的区分 Node3D *nSucc;

          // nSucc = nPred->createSuccessor(i);
          float obs_dis;
          // 这个距离是没有考虑车辆的大小的
          obs_dis = al.voronoi_al.getDistance((int)(nPred->getX()),(int)(nPred->getY()));

          // 扩展节点时候考虑与障碍物之间的距离
          nSucc = nPred->createSuccessor_conside_obs(i,obs_dis);
          // 计算节点距离 
          
          // // 写的变步长扩展后继 编译有效 但是运行是失败 估计是内存访问越界
           
          // set index of the successor
          // 设置后继节点编号  也没有
          iSucc = nSucc->setIdx(width, height);

          // 检查生成的扩展节点是不是在图内 并且该节点是可以到达的 而不是障碍物（包含了碰撞检测 很重要）
          //                                     碰撞检测 使用模板函数写的 包含3D 与 2D节点
          //TODO 注意：  但是这里 没有检测 两者之间是否可能存在障碍物 这个需要自行作出检查
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc))
          {

            // 确保生成的扩展节点不在clsoe集合内 或者扩展过 但是扩展节点 当前节点编号相同（扩展在了同一个珊格内）
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
            {

              // 计算扩展节点的 G值
              
              nSucc->updateG();
              
              // 考虑节点短期的障碍物代价   
              // 追加障碍物距离代价      
              nSucc->append_obs_g(obs_dis);

              // 获取该扩展节点的 G值
              newG = nSucc->getG();

              // 如果该后继节点已经在 open集合内了 或者 该新节点新的g值 小于之前的
              // 或者  如果扩展的节点和当前节点在同一个格子内
              // 那么需要更新 H 值 并检验重合效果优劣
              if (!nodes3D[iSucc].isOpen() ||
                  newG < nodes3D[iSucc].getG() ||
                  iPred == iSucc)
              {
                ros::Time s_cal_h = ros::Time::now();
                
                // 核心 计算该节点的 预估 H值代价
                updateH(*nSucc, goal,
                        nodes2D,
                        dubinsLookup,
                        width, height,
                        configurationSpace,
                        visualization);

                ros::Time e_cal_h = ros::Time::now();
                ros::Duration d_cal_h(e_cal_h - s_cal_h);
                write_time_dubins("cal_h_time_cost.txt",d_cal_h);


                // if the successor is in the same cell but the C value is larger
                // 如果扩展的节点和当前节点在同一个格子内 并且扩展的节点的代价较高
                // 则保留当前节点 舍弃扩展节点                                   0.01
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker)
                {
                  delete nSucc;
                  continue;
                }
                // 反之 保留扩展节点 舍弃当前节点
                // 更更新 当前节点的父亲节点
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker)
                {
                  nSucc->setPred(nPred->getPred());
                }
                // 父亲节点和子结点两个节点 相距很近并且方向角角差不多
                if (nSucc->getPred() == nSucc)
                {
                  std::cout << "looping";
                }

                // put successor on open list
                // 设置该节点在opne列表内  并将扩展节点 将如open 列表中
                nSucc->open();

                // 记录占用信息
                nodes3D[iSucc] = *nSucc;

                O.push(&nodes3D[iSucc]);
                // 析构该节点

                delete nSucc;
              }
              else
              {
                delete nSucc;
              }
            }
            else
            {
              delete nSucc;
            }
          }
          else
          {
            // 该扩展节点无效 舍弃该节点
            delete nSucc;
          }
        }
      }
    }

      // 写入每一次pop 一个节点的时间
     ros::Time suanfa_e = ros::Time::now();
     ros::Duration s_e_dubins(suanfa_e - suanfa_s);
     write_time_dubins("each_iter_cost.txt",s_e_dubins);

     
  }

  // open 集合为空 都找不到可行路径
  // 算法失败 返回空指针
  if (O.empty())
  {
    // 因为队列为空导致失败
    cout << "*****************************************" << endl;
    cout << "队列为空了，找不到路径，所以规划路径失败 " << endl;
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################

/// 返回在2D 节点扩展中 找到终点时的G值代价
// G值代价计算方式
float aStar(Node2D &start,
            Node2D &goal,
            Node2D *nodes2D,
            int width,
            int height,
            CollisionDetection &configurationSpace,
            Visualize &visualization)
{

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // 重置每一个2D节点的 h g值
  for (int i = 0; i < width * height; ++i)
  {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
   ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D *,
                             boost::heap::compare<CompareNodes>>
      O;

  // 更新2D节点的H值 当前起始节点到欧式距离的平方根
  start.updateH(goal);

  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);

  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D *nPred; // queue中当前拿出来的点
  Node2D *nSucc; // 当前队列最前点 的 可能临时后继节点 使用八向图遍历

  // continue until O empty
  while (!O.empty())
  {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // 该节点已经在闭合集合中了 不扩展该后继
    if (nodes2D[iPred].isClosed())
    {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // 扩展节点
    else if (nodes2D[iPred].isOpen())
    {
      // 加入 close集合 并设置已经被扩展过
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D)
      {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // 弹出该节点
      O.pop();

      // 判断是不会已经到达终点
      if (*nPred == goal)
      {
        // 找到了目标 返回达到目标点的G值
        return nPred->getG();
      }
      // 扩展节点
      else
      {
        // 创建后继节点  八向图
        for (int i = 0; i < Node2D::dir; i++)
        {
          // 创建可能的后继节点
          nSucc = nPred->createSuccessor(i);
          // 后继节点编号
          iSucc = nSucc->setIdx(width);

          // 确保后继节点的安全无碰撞 并且在地图范围内 并且不再close列表内
          if (nSucc->isOnGrid(width, height) &&
              configurationSpace.isTraversable(nSucc) &&
              !nodes2D[iSucc].isClosed())
          {
            // 计算 该节点G值 加上其父亲节点的G值
            // 最终等于扩展到该节点是 所有累积做过的路径节点的G总和
            nSucc->updateG();

            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            // 不在open 集合内 或者新的G值要小 加入到open集合中
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG())
            {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            }
            else
            {
              delete nSucc;
            }
          }
          else
          {
            delete nSucc;
          }
        }
      }
    }
  }

  // 返回1000 在没有查找到终点 也就是说该节点 扩展找不到终点 设置为1000的代价
  // 也就是避免走到该节点 将该节点 在队列中往后推
  return 1000;
}



//###################################################
//                                         COST TO GO
//###################################################
// 计算 H代价 一个点 到 另外一个点的huritic cost
// 当前节点 到 终点的预估代价
// 包括两大部分 考虑障碍物不考虑运动学 不考虑障碍物但考虑运动学
void updateH(Node3D &start, const Node3D &goal,
             Node2D *nodes2D,
             float *dubinsLookup,
             int width, int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization)
{

  // 以下两个为忽略障碍物 只考虑动力学的huritic cost
  // 采用dubins策略的代价
  float dubinsCost = 0;
  // 采用 reedshepp策略的代价
  float reedsSheppCost = 0;

  // 以下两个为忽略动力学 只考虑障碍物的huritic cost
  // 以2D node的A*算法 代价锚定
  float twoDCost = 0;
  // 同时考虑节点在珊格内的偏移量
  float twoDoffset = 0;

  // dubins 曲线 仅仅只能前进 不允许后退 reed-sheep 曲线则是允许前进后退的
  //  默认为false 意味着允许倒车 忽略该段if逻辑
  if (Constants::dubins)
  {

    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State *dbStart = (State *)dubinsPath.allocState();
    State *dbEnd = (State *)dubinsPath.allocState();

    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());

    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // 不考虑障碍物 但是不使用dubins方案 直接使用距离两点直线距离
  // dubinsCost = std::sqrt(start.distance(goal));

  // if reversing is active use a
  // 同时允许使用前进后退功能  计算考虑只障碍物代价的
  if (Constants::reverse && !Constants::dubins)
  {
    //    ros::Time t0 = ros::Time::now();
    //                                                 机器的最小转弯半径  6
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);

    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();

    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());

    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());

    // 计算reed_shep代价 也就是起点到终点的距离？？？
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);

    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

  // 2D的启发代价  不考虑障碍物
  // 打开了2D 启发代价开关  并且该节点没有被扩展过
  // 这种情况下 才计算代价该3D节点对应的2D节点的H代价
  if (Constants::twoD &&
      !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered())
  {
    //    ros::Time t0 = ros::Time::now();
    // 构建开始2D节点
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // 构建目标2D节点
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);

    // run 2d astar and return the cost of the cheapest path for that node
    // 2d 的A* 算法获取最小代价 并且设置该开始2d节点找到目标节点 需要总的G值代价
    //  也就是终点节点的G值  完全可以直接选择离线计算
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
        aStar(goal2d, start2d,
              nodes2D,
              width, height,
              configurationSpace,
              visualization));
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

  //  本质就是减少栅格内部位置的偏差影响
  if (Constants::twoD)
  {

    // 将两个位置 放在一个珊格内 计算偏差
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));

    // 该节点到达终点的G值 - 终点起点放在一个珊格内的偏移量
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
  }

  // return the maximum of the heuristics, making the heuristic admissable
  //                                           不使用dubins曲线时 dubinscost = 0
  // 最后该3D节点的 H代价 取考虑障碍物 和不考虑障碍物中的最大值
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));

  // cout << "reedsSheppCost: " << reedsSheppCost << " " << "dubinsCost: " << dubinsCost << " " << "twoDCost: " << twoDCost << endl;
}

//###################################################
//                                        DUBINS SHOT
//###################################################
// DUBINS SHOT算法
// 如果dubinsguo过长 可视为无效 进行重新采样规划
Node3D *dubinsShot(Node3D &start,
                   const Node3D &goal,
                   CollisionDetection &configurationSpace)
{

  // 起始点信息数组
  double q0[] = {start.getX(), start.getY(), start.getT()};
  // 终止点信息数组
  double q1[] = {goal.getX(), goal.getY(), goal.getT()};

  // initialize the path
  // typedef struct
  // {
  //     double qi[3];       // the initial configuration
  //     double param[3];    // the lengths of the three segments
  //     double rho;         // model forward velocity / model angular velocity
  //     int type;           // path type. one of LSL, LSR, ...
  // } DubinsPath;

  // dubins 算法 用于记录生成的最短的dubins曲线结果 这里是归一化的结果值
  DubinsPath path;

  // 计算dubins曲线算法 结果保留在path中
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  // 生成的 dubins 曲线的长度
  float length = dubins_path_length(&path);

  // 对length 进行约束 太长的lenthgh 可以考虑不采纳       
  
  float dis_s_e = std::sqrt(start.distance(goal));
  
  if(length > Constants::dubins_len_beishu * dis_s_e) {
      return nullptr;
  }   


  // 建立新的 3d 节点 包含了所有的节点 间隔1m一个节点                 1
  Node3D *dubinsNodes = new Node3D[(int)(length / Constants::dubinsStepSize) + 2];

  // x 初始值 1.0
  x += Constants::dubinsStepSize;

  // 1m 循环迭代 直到终点 每一步都要进行碰撞检测
  // 肯定会少一段 将少的一段补上
  while (x < length)
  {
    double q[3];

    // 在生成的dubins曲线上 采样 获取当前x对应采样点的 x y theta
    // 采样的结果保留在参数q中
    // 并利用该信息进行后面的碰撞检测
    dubins_path_sample(&path, x, q);

    // 设置dubins节点信息
    // 数组 内部是每一个node 3d节点
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // 碰撞检测 如果没有风险 检测下一个
    if (configurationSpace.isTraversable(&dubinsNodes[i]))
    {

      // 设置该节点的父亲节点
      if (i > 0)
      {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      }
      // 初始设置
      else
      {
        dubinsNodes[i].setPred(&start);
      }

      // 节点出现loop循环
      if (&dubinsNodes[i] == dubinsNodes[i].getPred())
      {
        std::cout << "looping shot";
      }

      // x 迭代 进行dubins曲线上下一个点的检测
      x += Constants::dubinsStepSize;

      i++;
    }
    /// 存在风险 直接杀死 取消这一次 dubins的计算
    //  并返回空指针
    else
    {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete[] dubinsNodes;
      return nullptr;
    }
  }

  // 判断最后一个采样点 离终点太远 则需要连接终点 
  if(dubinsNodes[i-1].distance(goal) > 0.09){
    // 添加节点
    dubinsNodes[i].setX(goal.getX());
    dubinsNodes[i].setY(goal.getY());
    dubinsNodes[i].setT(goal.getT());
    dubinsNodes[i].setPred(&dubinsNodes[i - 1]);

    i++;
  }
  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  // 成功找到满足条件的dubins曲线 直接返回最后一个dubins节点
  return &dubinsNodes[i - 1];
}

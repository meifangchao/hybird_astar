#include "planner.h"
#include <fstream>
using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################

void write_time(string file_name1,ros::Duration dur);

// planner类 构造函数
Planner::Planner()
{
  // _____
  // TODOS
  //    initializeLookups();map
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // 发布主题 rviz会订阅该主题
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // TOPICS TO SUBSCRIBE
  // 静态地图 为手动
  if (Constants::manual)
  {
    // 直接使用 map_server 功能包
    // 使用订阅话题 利用回调函数实现  Planner::setMap为函数接口
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  }
  
  else
  {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  // 设置起点和终点 通过节点句柄 然后消息发布
  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);

  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################

/// 初始化 碰撞检测查询表 以及 dubins查询表
void Planner::initializeLookups()
{
  if (Constants::dubinsLookup)
  {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
/// 设置 读取地图 并内部调用plan（）函数进行规划
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map)
{
  // 该开关 默认没有打开
  if (Constants::coutDEBUG)
  {
    std::cout << "I am seeing the map..." << std::endl;
  }

  
  // map_server节点提供的地图
  grid = map;
 
  //  根据当前地图 更新碰撞检测类中的地图
  configurationSpace.updateGrid(map);
  // create array for Voronoi diagram
  //   ros::Time t0 = ros::Time::now();

  // 读取地图尺寸
  int height = map->info.height;
  int width = map->info.width;

  ///生成二值地图信息 这里就是一个二维数组 写成了指针的指针的形式而以
  bool **binMap;

  // 这里最终没有释放这个指针
  binMap = new bool *[width];
  for (int x = 0; x < width; x++)
  {

    binMap[x] = new bool[height];
  }

  for (int x = 0; x < width; ++x)
  {
    for (int y = 0; y < height; ++y)
    {
      // 完全的二值信息 0 1
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  // 根据binMap地图信息 建立voronoi图
  // 初始化
  voronoiDiagram.initializeMap(width, height, binMap);
  // 更新
  voronoiDiagram.update();
  // 可视化
  voronoiDiagram.visualize();

  // Algorithm::voronoi_al = voronoiDiagram;

  // vor_forcost = voronoiDiagram;

  //  ros::Time t1 = ros::Time::now();
  //  ros::Duration d(t1 - t0);
  //  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  // 自动模式 忽略该模式
  // 该模式会在planner构造函数内 启动plan（）函数 达到效果
  if (!Constants::manual && 
      listener.canTransform("/map", ros::Time(0), 
      "/base_link", 
      ros::Time(0), 
      "/map", 
      nullptr))
  {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    // 判断是否是有效的起点 也就是判断起点是否在地图内
    if (grid->info.height >= start.pose.pose.position.y &&
        start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x &&
        start.pose.pose.position.x >= 0)
    {
      // set the start as valid and plan
      validStart = true;
    }
    else
    {
      validStart = false;
    }

    // 开始规划
    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
// 初始化起点 用于发布节点的函数入口
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial)
{

  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  // 箭头的方向角
  float t = tf::getYaw(initial->pose.pose.orientation);

  // 发布起始位置为 rviz
  geometry_msgs::PoseStamped startN;

  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  // 起点没有越界 才有效
  if (grid->info.height >= y && y >= 0 && 
      grid->info.width >= x && x >= 0)
  {
    validStart = true;
    start = *initial;

    if (Constants::manual)
    {
      plan();
    }
    // ros::Publisher pubStart; 建立publish对象
    // publish()函数可以在topic上发布(pubish)消息；  
    pubStart.publish(startN);
  }
  else
  {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr &end)
{
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0)
  {
    validGoal = true;
    goal = *end;

    if (Constants::manual)
    {
      plan();
    }
  }
  else
  {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
// 开始规划 
void Planner::plan()
{

  //  DynamicVoronoi share_vro;

  // if a start as well as goal are defined go ahead and plan
  // 具备起点和终点 才开始规划 防止起点终点越界地图的检查
  if (validStart && validGoal)
  {

    //
    int width = grid->info.width;
    int height = grid->info.height;

    // 72
    int depth = Constants::headings;
    cout << "地图宽：" << width << endl;
    cout << "地图高：" << height << endl;

    // 代表所有3D节点的个数 也就是SE3的状态可能的总数
    // 长 * 宽 * 72
    int length = width * height * depth;

    // define list pointers and initialize lists
    // 3D 节点的指针容器
    Node3D *nodes3D = new Node3D[length]();
    // 2D 节点的指针容器
    Node2D *nodes2D = new Node2D[width * height]();

    // ________________________
    // 检索终点位置 接收 rviz msg得到
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    // 获取 yaw值
    float t = tf::getYaw(goal.pose.orientation);

    // 较长的路径终点设置        
    // float x = 54.1023 / Constants::cellSize;
    // float y = 38.6037 / Constants::cellSize;
    // float t = 1.35213;

    // 验证局部障碍
    // float x = 47.8157 / Constants::cellSize;
    // float y = 43.0641 / Constants::cellSize;
    // float t = 0.463646;

    // float x = 71.2048 / Constants::cellSize;
    // float y = 21.7832 / Constants::cellSize;
    // float t = 0.226801;


    cout << "实际终点 x:" << x << " " << "y: " << y << "t: " << t << endl;
    // 将角度转换成  (0,2PI]
    t = Helper::normalizeHeadingRad(t);

    /// 初始化 目标节点
    const Node3D nGoal(x, y, t,
                       0, 0, // g h 值
                       // 父节点
                       nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);

    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);

    // 较长的路径起点设置
    //  x = 19.4221 / Constants::cellSize;
    //  y = 20.2843 / Constants::cellSize;
    //  t = 0.07709;
    

    //  x = 25.2051 / Constants::cellSize;
    //  y = 20.5319 / Constants::cellSize;
    //  t = -0.0370198;

    

    // // 出现离谱轨迹的情况
    //  x = 35.8071 / Constants::cellSize;
    //  y = 4.1266 / Constants::cellSize;
    //  t = 0.25877;


    cout << "实际起点 x: " << x << " " << "y: " << y << "t: " << t << endl;
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);

    // 初始化起始节点
    Node3D nStart(x, y, t, 0, 0, nullptr);

    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);

    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();

    // 清除之前的路径
    path.clear();
    // 平滑后的路径清除
    smoothedPath.clear();
    //
    // 计算前端时间 
    // HybridAStar::DynamicVoronoi voronoi_al = voronoiDiagram;

    // share_vro = voronoiDiagram;

    tool_a.voronoi_al = voronoiDiagram;

    // 使用 hybird A* 算法 搜索路径
    Node3D *nSolution = Algorithm::hybridAStar(nStart, nGoal,
                                               nodes3D, nodes2D,
                                               width, height,
                                               configurationSpace,
                                               dubinsLookup,
                                               visualization,
                                               tool_a);

    



    ros::Time t_qianduan = ros::Time::now();

    ros::Duration d_qianduan(t_qianduan - t0);
    //  std::cout << "前后段总共: TIME in ms: " << d * 1000 << std::endl;
    // 
    write_time("qianduan_time_cost",d_qianduan);

    // 由起点 回溯出hybird A*的求解结果
    // 可以将结果打印出来
    smoother.tracePath(nSolution);

    // smoother.getPath() 函数 用于求解递归轨迹  里面自带参数 返回的是node3d的vector
    // updatePath 函数 根据回溯找出所有的路径信息  设置可视化的信息 并发布出去
    path.updatePath(smoother.getPath());

    // 平滑轨迹  后端核心部分
    // 目前 有一些内部具体的梯度下降策略没有弄清楚 
    // 围绕 kappa 那一块
    // 后端用时
    // 初始化 smooth类的voro图
    smoother.smoothPath(voronoiDiagram);


    ros::Time t_houduan = ros::Time::now();
    ros::Duration d_houduan(t_houduan - t_qianduan);
    //  std::cout << "前后段总共: TIME in ms: " << d * 1000 << std::endl;
    // 
    write_time("hou_time_cost",d_houduan);

    // 对平滑后的结果 进行设置可视化的信息 并发布出去
    smoothedPath.updatePath(smoother.getPath());

    /// 算法计算时间
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "前后段总共: TIME in ms: " << d * 1000 << std::endl;
    // 
    write_time("total_time_cost",d);

    // _________________________________

    // 发布一系列车辆节点 车辆框图信息 以及节点代价等信息
    // 目前该部分 还没有细看完成
    // 平滑前的
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();

    // 平滑后的
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();

    // 发布代价节点信息
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    // 析构所有的节点
    delete[] nodes3D;
    delete[] nodes2D;
  }

  else
  {
    std::cout << "missing goal or start" << std::endl;
  }
}


void write_time(string file_name1,ros::Duration dur)
{
  // 1.包含头文件 fstream
  //      2.创建流对象
  ofstream ofs_xyt;
  
  ofs_xyt.open(file_name1, ios::app);
  
  ofs_xyt << dur * 1000 << endl; 

 
  ofs_xyt.close();
  
}
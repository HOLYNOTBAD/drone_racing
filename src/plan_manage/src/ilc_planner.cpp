#include <plan_manage/ilc_planner.h> // 包含 ilc_planner 的头文件,声明类和成员
#include <iostream> // 包含标准输入输出流，用于调试输出
#include <numeric> // 包含 accumulate 等数值算法
#include <nav_msgs/Odometry.h> // 包含 nav_msgs::Odometry 的消息类型定义
#include <nav_msgs/Path.h> // 包含 nav_msgs::Path 的消息类型定义

using namespace std; // 使用 std 命名空间以简化代码中的标准库前缀

// 将全局路径与机器人状态传入本地规划器，初始化并触发 ILC 学习
bool ilc_planner::setPlan(std::vector<Eigen::Vector3d> orig_global_plan, Eigen::Vector3d robot_pose, Eigen::Vector3d robot_vel) { // setPlan: 设置全局路径与当前机器人状态
  ROS_INFO("LocalPlanner setPlan"); // 打印 ROS 日志，指示进入 setPlan

  _global_plan.clear(); // 清空内部保存的全局路径
  _global_plan = orig_global_plan; // 将传入的路径拷贝到内部存储

  _robot_pose = robot_pose; // 保存当前机器人位姿
  _robot_vel = robot_vel; // 保存当前机器人速度

  double v_init = v_min + v_max / 3;       // 初始速度设为最小速度 v_min
  // double v_init = _robot_vel.norm();    // 也可以使用当前速度作为初始速度
  std::vector<double> v_list(_global_plan.size(),v_init); // 构造与路径等长的速度列表 v_list 并初始化为 v_init
  
  // double start = clock(); // 可选性能计时
  spatialILC(v_list, _iteration); // 调用空间 ILC 学习算法，对初始速度列表进行迭代优化
 
  return true; // 返回成功
}

void ilc_planner::set_param(double vmin, double vmax, double goalth, double kpvl, double kplaw, double kdlaw, std::vector<double> xth, double kppath, double kdpath, double tau, int iteration)
{
  v_max = vmax; // 速度上限
  v_min = vmin; // 速度下限
  goal_th = goalth; // 到达目标的阈值
  kp_vl = kpvl; // 学习律的比例增益
  kp_law = kplaw; // iterate 中使用的 kp
  kd_law = kdlaw; // iterate 中使用的 kd
  x_th = xth; // 每个点的阈值数组
  kp_path = kppath; // 路径误差比例增益
  kd_path = kdpath; // 路径误差微分增益
  _tau = tau; // 动力学模型中的时间常数（或响应系数）
  _iteration = iteration; // ILC 迭代次数
}


// 判断是否已经接近目标点
bool ilc_planner::judge_reach(){
  if (_global_plan.size()>0) // 若存在全局路径
  {
    Eigen::Vector3d goal_pose = _global_plan.back(); // 取最后一个路径点作为目标
    Eigen::Vector3d goal_vec = goal_pose - _robot_pose; // 目标向量
    double goal_dis = goal_vec.norm(); // 到目标的距离

    if (goal_dis < goal_th)
    {
      return true; // 小于阈值则认为已到达
    }
  }
  return false; // 否则未到达
}

// 在全局路径中找到离机器人最近的索引，并返回参考点与方向信息
bool ilc_planner::get_index(Eigen::Vector3d& refer_pose, Eigen::Vector3d& move_direction, Eigen::Vector3d& move_track, int& index)
{
  std::vector<Eigen::Vector3d>::iterator it = _global_plan.begin(); // 迭代器指向路径起点
  
  int ii = 0; // 索引计数
  index = 0; // 输出索引初始化

  double min = 10000; // 初始化最小距离为较大值

  while (_global_plan.size()>2 && it != _global_plan.end()-1) // 遍历路径直到倒数第二个点
  {
      Eigen::Vector3d robot_dis =  *it - _robot_pose; // 当前路径点到机器人位置的向量
      double dist_sq = robot_dis.norm(); // 距离
      if (dist_sq < min)
      {
          min = dist_sq;        
          index = ii; // 更新最近点索引

          refer_pose = *it; // 更新参考点

          std::vector<Eigen::Vector3d>::iterator itt  =  it; // 复制迭代器
          ++itt; // 指向下一个点
          
          move_direction = *itt - *it; // 计算切向方向（下一个点 - 当前点）
          move_direction = move_direction.normalized(); // 单位化

          move_track = robot_dis; // 横向偏差向量（当前点到机器人）
      }
      ++it; // 下一点
      ii++; // 索引递增
  }
  return true; // 成功返回
}

// 裁剪全局路径中机器人身后的点（保留未来部分）
bool ilc_planner::pruneGlobalPlan(double dist_behind_robot)
{
  if (_global_plan.empty()) return true; // 若路径为空直接返回

  double dist_thresh_sq = dist_behind_robot * dist_behind_robot; // 阈值平方

  std::vector<Eigen::Vector3d>::iterator it        = _global_plan.begin(); // 起始迭代器
  std::vector<Eigen::Vector3d>::iterator erase_end = it; // 待删除区间的结束迭代器

  while (it != _global_plan.end())
  {   
    Eigen::Vector3d robot_dis = *it - _robot_pose; // 路径点到机器人向量

    double dist_sq = robot_dis.norm(); // 距离
    if (dist_sq < dist_thresh_sq)
    {
        erase_end = it; // 找到第一个小于阈值的点
        break; // 停止遍历
    }
    ++it; // 否则继续
  }

  if (erase_end == _global_plan.end()) return false; // 未找到可删区间，返回 false

  if (erase_end != _global_plan.begin()) _global_plan.erase(_global_plan.begin(), erase_end); // 删除起始到 erase_end 的点
  if (_global_plan.end() - _global_plan.begin()>100) _global_plan.erase(_global_plan.begin()+99, _global_plan.end()); // 若路径过长则裁剪为不超过 100 个点
  // if (erase_begin != global_plan.end()) global_plan.erase(erase_begin, global_plan.end());

  return true; // 裁剪成功
}

// 在给定位置 p 上，找到最近的轨迹参考点并返回相关信息：refer_pose、move_direction、move_track、error、current
void ilc_planner::getpoint(Eigen::Vector3d p, Eigen::Vector3d &refer_pose, Eigen::Vector3d &move_direction, Eigen::Vector3d &move_track, double &error, int &current)
{
  int path_length = _global_plan.size(); // 路径长度
  Eigen::Vector3d p_des; // 当前候选参考点
  Eigen::Vector3d next_p_des; // 下一个参考点

  Eigen::Vector3d dp; // 候选点到 p 的向量

  double min = 1000; // 初始化最小距离

  // cout<< "current: " << current << endl;

  for (int i = current; i<path_length-1; i++) // 从 current 开始搜索直到倒数第二个点
  {
    p_des = _global_plan[i]; // 取第 i 个路径点
    dp = p_des - p; // 计算与 p 的向量差
       
    if (dp.norm() < min) // 若更接近则更新
    {
      error = dp.norm(); // 更新误差为到该点的距离
      min = error; // 更新最小值
      current = i; // 更新 current 索引为 i
      refer_pose = p_des; // 设置参考点
      move_track = dp; // 横向向量为 dp
      next_p_des = _global_plan[i+1]; // 下一个参考点

      move_direction = (next_p_des - p_des).normalized(); // 切向方向为下一个点减当前点并单位化
    }
  }


}

// 对速度向量进行饱和处理，使其范数不超过 vmax
void ilc_planner::saturation(Eigen::Vector3d &v, double vmax)
{
  double value = v.norm(); // 计算当前速度范数
  if (value > vmax)
  {
    v = v/value*vmax; // 缩放到 vmax
  }
}




// 空间 ILC 学习算法的实现：基于给定速度序列 v_list，进行多次迭代优化
bool ilc_planner::spatialILC(std::vector<double>& v_list, int iteration)
{
  vel_list.clear(); // 清空内部速度列表
  vel_list = v_list; // 将传入的速度列表复制到成员变量 vel_list

  // std::vector<geometry_msgs::PoseStamped>::iterator it        = _global_plan.begin();
  int path_length = v_list.size(); // 路径点数量
  // std::cout << path_length << std::endl;
  // std::vector<double> pathx(path_length);
  // std::vector<double> pathy(path_length);
  // std::vector<double> pathz(path_length);
  std::vector<double> error_list(path_length); // 与路径长度对应的误差缓存

  // for (int j = 0; j<path_length; j++)
  // {
  //   pathx[j] = _global_plan[j].pose.position.x;
  //   pathy[j] = _global_plan[j].pose.position.y;
  //   pathz[j] = _global_plan[j].pose.position.z;
  // }

  int current_all = path_length-1; // 最大索引（用于循环边界）

  Eigen::Vector3d p; // 当前模拟位置
  Eigen::Vector3d v; // 当前模拟速度

  Eigen::Vector3d v_para; // 期望沿轨迹方向的速度分量
  Eigen::Vector3d v_perp; // 期望垂直于轨迹的修正速度分量
  Eigen::Vector3d v_des; // 目标合成速度

  Eigen::Vector3d refer_pose; // 引用的轨迹点
  Eigen::Vector3d move_direction; // 轨迹切向方向
  Eigen::Vector3d move_track; // 从当前位置到参考点的向量（横向误差方向）

  // double kp = 0.6;
  // double kd = 1.5;

  // double vmax = 3;
  
  for (int i = 0; i<iteration; i++) // ILC 主循环，执行指定次数的迭代学习
  { 
    int k = 0; // 内部步数计数器
    int l = 0; // 当前参考索引
    int last_l = 0; // 上一次参考索引
    double error; // 当前误差
    double last_error; // 上一次误差
    
    int current = 0; // 当前路径搜索起点索引
    p = _robot_pose; // 初始化模拟位置为当前机器人位置
    v = _robot_vel; // 初始化模拟速度为当前机器人速度

    while (current < current_all-int(current_all/100)-1 && k < path_length) // 在路径上逐步前进直到末尾或达到步数上限
    {
      last_l = l; // 记录上一次索引
      last_error = error; // 记录上一次误差
      // double start = clock();

      getpoint(p, refer_pose, move_direction, move_track, error, current); // 找到最近的参考点和误差等信息
      // cout << current;
      // cout << " ";
      // double end = clock();
      // double last = end -start;
      // std::cout << last << std::endl;
      // std::cout << p[0] << std::endl;
      // std::cout << p[1] << std::endl;
      // std::cout << "current: "<< current << std::endl;
      // std::cout <<  "error: " << error << std::endl;
      // std::cout << move_direction[0] << std::endl;
      // std::cout << move_direction[1] << std::endl;
      l = current; // 更新当前索引

      if (l > last_l) // 如果索引有推进，用之前的误差填充区间
      {
        for (int m=last_l; m<l; m++)
        {
          error_list[m] = error; // 将当前误差赋值给区间内点
        }
      }

      v_para = move_direction * vel_list[current]; // 沿轨迹方向的速度分量 = 轨迹方向 * 对应速度
      v_perp = move_track * kp_path + (error- last_error) / (error+0.00001) * move_track * kd_path; // 横向校正项（包含比例与差分项）

      //曲率补偿
      Eigen::Vector3d curvature = getCurvature(current);
      double curvature_effect = curvature.norm();
      if (curvature_effect > 0.0001) // 避免除零
      {
        v_perp += move_direction.cross(curvature).normalized() * curvature_effect * vel_list[current] * 3.0;
      }



      v_des = v_para+v_perp; // 合成目标速度

      saturation(v_des, v_max); // 对速度进行饱和约束
      dynamic(v_des, v, p); // 根据动力学模型更新模拟速度与位置

      k++; // 步数计数
    }
    // cout << " " <<endl;
    // for (auto i = 0; i != error_list.size(); ++i) cout << error_list[i];
    // cout << " " <<endl;
    iterate(error_list); // 根据误差序列更新 vel_list（学习步骤）
    // std::cout << "leanrning 1 iteration" << endl;
    // for (int kkk=0; kkk<error_list.size(); kkk++) std::cout << error_list[kkk] << endl;
  }
  // cout << " " <<endl;
  // for (auto i = 0; i != vel_list.size(); ++i) cout << vel_list[i];
  // cout << " " <<endl;
  return true; // 表示学习过程完成
}


// 简单动力学模型：通过一阶闭环模型将期望速度转换为实际速度和位置的更新
void ilc_planner::dynamic(Eigen::Vector3d vdes, Eigen::Vector3d &v, Eigen::Vector3d &p)
{
  Eigen::Vector3d a; // 加速度
  a = _tau*(vdes-v); // 一阶差分的加速度：比例于速度误差
  double dt = 0.05; // 时间步长（固定）
  v = v+dt*a; // 更新速度
  p = p+dt*v; // 更新位置
}


// 根据误差序列执行一次 ILC 的参数更新（vel_list 的调整）
void ilc_planner::iterate(std::vector<double> error_list)
{
  // double kp_law = 1;
  // double kd_law = 0.2;
  // double kp_vl = 0.2;

  // double xth = 1.5;

  double temp; // 临时变量用于计算更新量
  if (error_list.size()>1) // 至少需要两个点才能计算差分项
  {
    for (int i = 0; i < error_list.size()-1; i++)
    {
      temp = kp_vl * ( 1 / ( 1 + exp(-kp_law*error_list[i]-kd_law*(error_list[i+1]-error_list[i])+x_th[i]) ) -0.5); // 非线性变换得到学习量
      vel_list[i] = vel_list[i] - temp; // 更新速度表
    }
    // std::cout << vel_list[0] << std::endl;
    // std::cout << vel_list.size() << std::endl;

    for (int i = 0; i < vel_list.size()-1; i++) 
    { 
      if (vel_list[i]<v_min) // 速度下限保护
      {
        vel_list[i]=v_min;
      }
    }
  }
}

// 根据机器人当前位姿计算要发布的速度命令 cmd_vel，返回参考点 refer_pose 与期望偏航 yaw_des
bool ilc_planner::computeVelocityCommands(Eigen::Vector3d &cmd_vel, Eigen::Vector3d &refer_pose, double &yaw_des, Eigen::Vector3d robot_pose) 
{
  // ROS_INFO("LocalPlanner computeVelocityCommands"); // 可选日志

  Eigen::Vector3d move_direction; // 沿轨迹切向方向
  Eigen::Vector3d move_track; // 横向偏差方向

  int index = 0; // 当前参考点索引

  get_v_list(robot_pose, refer_pose, move_direction, move_track, index); // 得到索引与方向信息

  Eigen::Vector3d robot_dis = refer_pose - robot_pose; // 机器人到参考点的向量
  double error = robot_dis.norm(); // 距离误差
  // double dt = 0.05;
  cmd_vel = vel_list[index]*move_direction + move_track*(kp_path*error + kd_path*(error-_last_error)); // 速度由前向速度与横向修正组成
  saturation(cmd_vel, v_max); // 对速度进行饱和

  _last_error = error; // 保存误差用于下次微分项计算
  yaw_des = atan2(move_direction[1], move_direction[0]); // 计算期望偏航角（朝向切向方向）

  // double k_yaw = 0.5;

  // double diff_yaw = yaw_des - yaw;
  // if (diff_yaw < 0)
  //   {diff_yaw += 2*3.14159265;}
  // if (diff_yaw > 3.14159265)
  //   {diff_yaw -= 2*3.14159265;}
  // cmd_vel.angular.z = k_yaw*diff_yaw;
  
  return true; // 成功计算速度命令
}

// 根据机器人位姿选择参考点并返回对应的速度表索引以及方向信息
bool ilc_planner::get_v_list(Eigen::Vector3d robot_pose, Eigen::Vector3d &refer_pose, Eigen::Vector3d& move_direction, Eigen::Vector3d& move_track, int& index) 
{
  std::vector<Eigen::Vector3d>::iterator it = _global_plan.begin(); // 迭代器从头开始
  
  int ii = 0; // 索引计数
  index = 0; // 初始化输出索引

  double min = 10000; // 初始化最小距离

  while (_global_plan.size()>2 && it != _global_plan.end()-1) // 遍历直到倒数第二点
  {
      Eigen::Vector3d robot_dis =  *it - robot_pose; // 当前路径点到机器人位置向量
      double dist_sq = robot_dis.norm(); // 距离
      if (dist_sq < min)
      {
          min = dist_sq;        
          index = ii; // 更新最小距离的索引
          refer_pose = *it; // 更新参考点

          std::vector<Eigen::Vector3d>::iterator itt  =  it; // 复制迭代器
          ++itt; // 指向下一点
          
          move_direction = *itt - *it; // 计算切向
          move_direction = move_direction.normalized(); // 单位化

          Eigen::Vector3d z; // 临时向量
          z = move_direction.cross(robot_dis); // 叉乘得到法向（未使用，保留用于后续逻辑）

          // move_track = robot_dis;
          // move_track = z.cross(move_direction);
          move_track = robot_dis.normalized(); // 横向轨迹向量归一化
      }
      ++it; // 下一个点
      ii++; // 递增索引
  }
  return true; // 成功返回
}

Eigen::Vector3d ilc_planner::getCurvature(int l)
{
  int path_length = _global_plan.size();

  // 如果索引越界或点数不足，返回零向量
  if (path_length < 3 || l < 1 || l >= path_length - 1) {
    return Eigen::Vector3d::Zero();
  }

  // 使用中心差分计算一阶导数（速度向量）和二阶导数（加速度向量）
  Eigen::Vector3d p_prev = _global_plan[l - 1];
  Eigen::Vector3d p_curr = _global_plan[l];
  Eigen::Vector3d p_next = _global_plan[l + 1];

  Eigen::Vector3d d1 = 0.5 * (p_next - p_prev); // 一阶导
  Eigen::Vector3d d2 = p_next - 2.0 * p_curr + p_prev; // 二阶导

  double d1_norm = d1.norm();
  if (d1_norm < 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  // 计算曲率向量 k = (v x a) x v / |v|^4
  // 注意：方向指向圆心（法向），模长为曲率 kappa
  Eigen::Vector3d cross_prod = d1.cross(d2);
  Eigen::Vector3d curvature_vec = cross_prod.cross(d1) / pow(d1_norm, 4);

  return curvature_vec;
}

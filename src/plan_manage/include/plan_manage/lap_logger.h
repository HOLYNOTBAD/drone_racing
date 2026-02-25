#ifndef PLAN_MANAGE_LAP_LOGGER_H
#define PLAN_MANAGE_LAP_LOGGER_H

#include <fstream>
#include <string>
#include <ros/time.h>

class LapLogger {
public:
  LapLogger();
  ~LapLogger();

  // 初始化日志文件路径（单一 CSV，包含所有圈）
  void init(const std::string& file_path);

  // 一圈开始时调用（进入 EXEC_TRAJ），lap_id 可用 lap_cnt_ 或 current_lap+1
  void startLap(int lap_id, const ros::Time& start_time);

  // 一圈结束时调用（lap_cnt_ 递增）
  void endLap();

  // 更新当前指令速度/推力（来自 speed_pub/thr_pub）
  void updateCommand(double speed, double thrust);

  // 更新当前真实速度/推力（来自 real_speed_pub/real_thr_pub）
  void updateReal(double speed, double thrust);

  // 在 odom 回调中调用，用于记录一个采样点（仅 EXEC_TRAJ 时真正写入）
  void logSample(const ros::Time& stamp, int fsm_state);

private:
  std::ofstream file_;
  bool enabled_;
  int current_lap_id_;
  ros::Time lap_start_time_;

  double cmd_speed_;
  double cmd_thr_;
  double real_speed_;
  double real_thr_;
};

#endif // PLAN_MANAGE_LAP_LOGGER_H


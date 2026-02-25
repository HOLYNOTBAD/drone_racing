#include <plan_manage/lap_logger.h>

#include <iomanip>

LapLogger::LapLogger()
    : enabled_(false),
      current_lap_id_(0),
      cmd_speed_(0.0),
      cmd_thr_(0.0),
      real_speed_(0.0),
      real_thr_(0.0) {}

LapLogger::~LapLogger() {
  if (file_.is_open()) {
    file_.close();
  }
}

void LapLogger::init(const std::string& file_path) {
  if (file_.is_open()) {
    file_.close();
  }
  file_.open(file_path.c_str(), std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    enabled_ = false;
    return;
  }

  // 写入表头：lap_id, t, cmd_speed, cmd_thrust, real_speed, real_thrust
  file_ << "lap_id,t,cmd_speed,cmd_thrust,real_speed,real_thrust\n";
  enabled_ = true;
}

void LapLogger::startLap(int lap_id, const ros::Time& start_time) {
  if (!enabled_ || !file_.is_open()) return;
  current_lap_id_ = lap_id;
  lap_start_time_ = start_time;
}

void LapLogger::endLap() {
  // 目前按圈结束不需要特殊处理，文件保持打开，继续记录后续圈
}

void LapLogger::updateCommand(double speed, double thrust) {
  cmd_speed_ = speed;
  cmd_thr_ = thrust;
}

void LapLogger::updateReal(double speed, double thrust) {
  real_speed_ = speed;
  real_thr_ = thrust;
}

void LapLogger::logSample(const ros::Time& stamp, int fsm_state) {
  if (!enabled_ || !file_.is_open()) return;
  if (fsm_state != 3) return;  // 仅在 EXEC_TRAJ 记录
  if (lap_start_time_.isZero()) return;

  double t = (stamp - lap_start_time_).toSec();

  file_ << std::fixed << std::setprecision(6)
        << current_lap_id_ << ","
        << t << ","
        << cmd_speed_ << ","
        << cmd_thr_ << ","
        << real_speed_ << ","
        << real_thr_ << "\n";
}


#pragma once

#include <memory>
#include <vector>

struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double kappa = 0.0;
  double velocity = 0.0;
  double angular_velocity = 0.0;
  double acceleration = 0.0;

  // 规划起点
  double planning_init_x = 0.0;
  double planning_init_y = 0.0;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  double target_curv = 0.0;

  double vx = 0.0;
  double vy = 0.0;

  // added
  double start_point_x = 0.0;
  double start_point_y = 0.0;

  double relative_x = 0;
  double relative_y = 0;

  double relative_distance = 0;

  double last_velocity = 0;
  
  double last_v_err = 0;
  double cur_v_err = 0;

  double last_v_time = 0;
  double cur_v_time = 0;

  double last_acc = 0;
  double cur_acc = 0;
};

struct TrajectoryPoint {
  double x;
  double y;
  double heading;
  double kappa;
  double v;
  double a;
};

struct TrajectoryData {
  std::vector<TrajectoryPoint> trajectory_points;
};

struct LateralControlError {
  double lateral_error;
  double heading_error;
  double lateral_error_rate;
  double heading_error_rate;
};

struct ControlCmd {
  double steer_target;
  double acc;
};

typedef std::shared_ptr<LateralControlError> LateralControlErrorPtr;
#include "mpc_controller.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

void Check(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool Near(double a, double b, double tolerance = 1e-8) {
  return std::abs(a - b) < tolerance;
}

class TestController : public shenlan::control::MPCController {
 public:
  shenlan::control::Matrix References(const std::vector<TrajectoryPoint> &points, double speed) {
    trajectory_points_ = points;
    return BuildReferenceStates(0, speed);
  }
};

}  // namespace

int main() {
  TestController controller;
  controller.Init();
  const std::vector<TrajectoryPoint> curve{
      {0.0, 0.0, 0.0, 0.0, 5.0, 0.0},
      {0.1, 0.0, 0.0, 0.0, 5.0, 0.0},
      {0.1, 0.0, 0.0, 0.0, 5.0, 0.0},
      {0.5, 0.0, 0.4, 0.8, 4.0, 0.0}};
  const auto reference = controller.References(curve, 5.0);
  Check(reference.cols() == 11, "expected one reference per predicted state");
  Check(Near(reference(2, 1), 0.0), "reference must interpolate along the first segment");
  Check(Near(reference(2, 6), 0.2), "reference must interpolate future heading");
  Check(Near(reference(3, 6), 2.0), "reference must include future yaw rate");
  Check(Near(reference(5, 6), 0.5), "reference must include future speed");
  Check(Near(reference(2, 10), 0.4), "reference must reach the end of the trajectory");
  Check(Near(reference(3, 10), 0.0), "held endpoint must have zero yaw rate");

  const auto stationary = controller.References(curve, 0.0);
  Check(stationary.isZero(), "stationary preview should remain at the initial reference");
  const auto ending = controller.References({curve.front()}, 5.0);
  Check(ending.isZero(), "preview at the endpoint should hold its reference");

  const std::vector<TrajectoryPoint> wrapped{
      {0.0, 0.0, 3.13, 0.0, 5.0, 0.0},
      {0.5, 0.0, -3.13, 0.0, 5.0, 0.0}};
  const auto wrapped_reference = controller.References(wrapped, 5.0);
  Check(std::abs(wrapped_reference(2, 5)) < 0.02, "heading interpolation must cross the angle wrap");

  VehicleState state{};
  state.velocity = 5.0;
  TrajectoryData straight{{curve[0], curve[1], {0.5, 0.0, 0.0, 0.0, 5.0, 0.0}}};
  TrajectoryData turning{{
      curve[0], curve[1],
      {0.3, 0.02, 0.2, 0.8, 5.0, 0.0},
      {0.5, 0.08, 0.4, 0.8, 5.0, 0.0}}};
  TrajectoryData right_turn = turning;
  for (auto &point : right_turn.trajectory_points) {
    point.y = -point.y;
    point.heading = -point.heading;
    point.kappa = -point.kappa;
  }
  ControlCmd straight_cmd{};
  ControlCmd turning_cmd{};
  ControlCmd right_cmd{};
  Check(controller.ComputeControlCommand(state, straight, straight_cmd), "straight solve failed");
  Check(controller.ComputeControlCommand(state, turning, turning_cmd), "turning solve failed");
  Check(controller.ComputeControlCommand(state, right_turn, right_cmd), "right turn solve failed");
  Check(turning_cmd.steer_target > straight_cmd.steer_target + 1e-5,
        "future left bend must change steering before reaching it");
  Check(right_cmd.steer_target < straight_cmd.steer_target - 1e-5,
        "future right bend must change steering before reaching it");
  Check(!controller.ComputeControlCommand(state, TrajectoryData{}, turning_cmd),
        "empty trajectory must be rejected");
}

#include <vector>
#include <manipulator/controller/i_arm_controller.h>

namespace manipulator::controller {
class SmoothPositionController : public IArmController {
 public:
  SmoothPositionController() = default;
  virtual ~SmoothPositionController() = default;

  JointCommand Compute(
      const JointStates& joint_states,
      const JointSetPoint& joint_setpoint,
      double dt) override;
 private:
  std::vector<double> pos_set_;
};
} // namespace manipulator::controller
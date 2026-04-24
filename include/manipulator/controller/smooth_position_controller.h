#include <vector>
#include <manipulator/controller/i_arm_controller.h>

namespace manipulator::controller {
class SmoothPositionController : public IArmController {
 public:
  SmoothPositionController();
  virtual ~SmoothPositionController() = default;
  void SetKpKd(const std::vector<double>& kp, const std::vector<double>& kd);

  JointCommand Compute(
      const JointStates& joint_states,
      const JointSetPoint& joint_setpoint,
      double dt) override;
 private:
  std::vector<double> pos_set_;
  std::vector<double> kp_;
  std::vector<double> kd_;
};
} // namespace manipulator::controller
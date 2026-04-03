#include <manipulator/arm/arm_factory.h>
#include <manipulator/arm/a_l1_beta.h>
#include <manipulator/arm/a_l1_gamma.h>
#include <stdexcept>

namespace manipulator::arm {

ArmFactory::ArmFactory() {
  arms_.emplace("a_l1_beta", std::make_unique<AL1Beta>());
  arms_.emplace("a_l1_gamma", std::make_unique<AL1Gamma>());
}

ArmFactory& ArmFactory::Instance() {
  static ArmFactory instance;
  return instance;
}

AbsArm::UniPtr ArmFactory::Create(const std::string& arm_name) {
  auto it = arms_.find(arm_name);
  if (it == arms_.end()) {
    throw std::invalid_argument("Unknown arm name: " + arm_name);
  }
  return std::move(it->second);
}

}  // namespace manipulator::arm

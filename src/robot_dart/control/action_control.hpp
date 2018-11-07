#ifndef ROBOT_DART_CONTROL_ACTION_CONTROL
#define ROBOT_DART_CONTROL_ACTION_CONTROL

#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/utils.hpp>

namespace robot_dart {
    namespace control {

        class ActionControl : public RobotControl {
        public:
            ActionControl() : RobotControl() {}
            ActionControl(const std::vector<double>& ctrl, bool full_control = false) : RobotControl(ctrl, full_control) {}

            void configure()
            {
                if (_ctrl.size() == _control_dof)
                    _active = true;
            }

            Eigen::VectorXd calculate(double)
            {
                ROBOT_DART_ASSERT(_control_dof == _ctrl.size(), "ActionControl: Controller parameters size is not the same as DOFs of the robot", Eigen::VectorXd::Zero(_control_dof));
                Eigen::VectorXd commands = Eigen::VectorXd::Map(_ctrl.data(), _ctrl.size());

                return commands.tail(_control_dof);
            }

            std::shared_ptr<RobotControl> clone() const
            {
                return std::make_shared<ActionControl>(*this);
            }

        };
    } // namespace control
} // namespace robot_dart

#endif

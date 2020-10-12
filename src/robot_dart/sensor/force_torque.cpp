#include "force_torque.hpp"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>

namespace robot_dart {
    namespace sensor {
        ForceTorque::ForceTorque(dart::dynamics::Joint* joint, size_t frequency, const std::string& direction) : Sensor(frequency), _direction(direction)
        {
            attach_to_joint(joint, Eigen::Isometry3d::Identity());
        }

        void ForceTorque::init()
        {
            _force.setZero();
            _torque.setZero();

            attach_to_joint(_joint_attached, Eigen::Isometry3d::Identity());
            _active = true;
        }

        void ForceTorque::calculate(double t)
        {
            if (!_attached_to_joint)
                return; // cannot compute anything if not attached to a joint

            Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
            auto child_body = _joint_attached->getChildBodyNode();
            // ROBOT_DART_ASSERT(child_body != nullptr, "Child BodyNode is nullptr", {});
            if (child_body)
                F2 = -dart::math::dAdT(_joint_attached->getTransformFromChildBodyNode(), child_body->getBodyForce());

            // We always compute things in SENSOR frame (aka joint frame)
            if (_direction == "parent_to_child") {
                _force = F2.tail(3);
                _torque = F2.head(3);
            }
            else // "child_to_parent" (default)
            {
                _force = -F2.tail(3);
                _torque = -F2.head(3);
            }

            Eigen::Matrix3d R = _joint_attached->getTransformFromChildBodyNode().linear().transpose();
            _force = R * _force;
            _torque = R * _torque;
        }

        std::string ForceTorque::type() const { return "ft"; }

        const Eigen::Vector3d& ForceTorque::force() const
        {
            return _force;
        }

        const Eigen::Vector3d& ForceTorque::torque() const
        {
            return _torque;
        }
    } // namespace sensor
} // namespace robot_dart
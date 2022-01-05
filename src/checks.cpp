//
// Created by george on 1/4/22.
//

#include <ros2_trajectory_utilities/checks.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_utilities");

namespace trajectory_utilities {
    double distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b) {
        Eigen::Translation<double, 3> t_a, t_b, t_diff;

        tf2::fromMsg(a.position, t_a.translation());
        tf2::fromMsg(b.position, t_b.translation());

        t_diff = Eigen::Translation<double, 3>(t_b.translation() - t_a.translation());
        return t_diff.translation().norm();
    }

    double angular_distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b) {
        Eigen::Quaterniond q_a, q_b, q_diff;
        tf2::fromMsg(a.orientation, q_a);
        tf2::fromMsg(b.orientation, q_b);

        q_diff = q_a.inverse() * q_b;

        return Eigen::AngleAxisd(q_diff).angle();
    }

    double distance_along_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &trajectory) {
        double total_dist = 0;
        geometry_msgs::msg::PoseStamped p_start, p_end;
        for (ulong i = 0; i < trajectory.size() - 1; i++) {
            p_start = trajectory[i];
            p_end = trajectory[i + 1];
            total_dist += distance_between_poses(p_start.pose, p_end.pose);
        }
        return total_dist;
    }

    bool check_joint_velocities(moveit::core::RobotModelConstPtr robot_model,
                                const std::map<std::string, double> &joint_diff) {
        const moveit::core::JointModel *joint_model;
        for (const auto &[joint_name, joint_velocity] : joint_diff) {
            // Find the joint model using the name and check the velocity against the limits
            joint_model = robot_model->getJointModel(joint_name);
            if (!joint_model->satisfiesVelocityBounds(&joint_velocity)) {
                RCLCPP_WARN_STREAM(LOGGER, "Joint velocity limit exceeded by " << joint_name <<
                                                                               " requested velocity " << joint_velocity
                                                                               << " with a limit of "
                                                                               << joint_model->getVariableBounds()[0].max_velocity_);
                return false;
            }
        }
        return true;
    }
}
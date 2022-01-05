//
// Created by george on 1/4/22.
//

#include <ros2_trajectory_utilities/moveit_plans.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_utilities");
namespace trajectory_utilities {
    bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                      moveit::planning_interface::MoveGroupInterface::Plan &second) {
        // Adding each point from the second trajectory to the first offsetting the time from start
        if (first.trajectory_.joint_trajectory.joint_names.size() ==
            second.trajectory_.joint_trajectory.joint_names.size()) {
            auto base_end_time = first.trajectory_.joint_trajectory.points.back().time_from_start;
            auto first_size = first.trajectory_.joint_trajectory.points.size();

            second.trajectory_.joint_trajectory.points.erase(second.trajectory_.joint_trajectory.points.begin());
            auto second_size = second.trajectory_.joint_trajectory.points.size();

            for (auto &point : second.trajectory_.joint_trajectory.points) {
                point.time_from_start.sec += base_end_time.sec;
                point.time_from_start.nanosec += base_end_time.nanosec;
                first.trajectory_.joint_trajectory.points.push_back(point);
            }
            if (first.trajectory_.joint_trajectory.points.size() != first_size + second_size) {
                RCLCPP_WARN_STREAM(LOGGER,
                                   "Combined trajectory size (" << first.trajectory_.joint_trajectory.points.size() <<
                                                                "is not the sum of the first and second (" << first_size
                                                                << " and " << second_size << ")");
            }
            return first.trajectory_.joint_trajectory.points.size() == first_size + second_size;
        } else {
            RCLCPP_WARN_STREAM(LOGGER, "Not the same number of joint names in first as the second");
            return false;
        }
    }

    bool retime_trajectory_constant_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                             moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                             std::string &jmg_name, std::string &end_effector_link,
                                             moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan) {

        const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(jmg_name);
        Eigen::Isometry3d p_start, p_end;
        double distance, time_between_points;
        rclcpp::Duration new_time_from_start = rclcpp::Duration(0, 0);
        retimed_plan.trajectory_ = plan.trajectory_;
        retimed_plan.planning_time_ = plan.planning_time_;
        retimed_plan.start_state_ = plan.start_state_;

        for (ulong i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {
            if (i > 0) {
                robot_state->setJointGroupPositions(joint_model_group,
                                                    plan.trajectory_.joint_trajectory.points[i - 1].positions);
                p_start = robot_state->getGlobalLinkTransform(end_effector_link);

                robot_state->setJointGroupPositions(joint_model_group,
                                                    plan.trajectory_.joint_trajectory.points[i].positions);
                p_end = robot_state->getGlobalLinkTransform(end_effector_link);

                distance = (p_end.translation() - p_start.translation()).norm();
                time_between_points = distance / desired_velocity;

                new_time_from_start =
                        rclcpp::Duration(retimed_plan.trajectory_.joint_trajectory.points[i - 1].time_from_start) + \
                    rclcpp::Duration::from_seconds(time_between_points);

                RCLCPP_DEBUG_STREAM(LOGGER, "Point " << i << " of " << plan.trajectory_.joint_trajectory.points.size()
                                                     << "\nOriginal time from start: " << rclcpp::Duration(
                        retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start).seconds()
                                                     << "\nDistance between points: " << distance
                                                     << "\nRecalculated time from start: "
                                                     << new_time_from_start.seconds() << std::endl);

                retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start = new_time_from_start;
            } else {
                // TODO: take the start state and determine distance from start to first point.

            }

        }
        return true;
    }

    bool retime_trajectory_trapezoidal_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                                moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                                double desired_acceleration, std::string &jmg_name,
                                                std::string &end_effector_link,
                                                moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan) {

        const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(jmg_name);
        Eigen::Isometry3d p_start, p_end;
        double distance, time_between_points;
        rclcpp::Duration new_time_from_start = rclcpp::Duration(0, 0);
        retimed_plan.trajectory_ = plan.trajectory_;
        retimed_plan.planning_time_ = plan.planning_time_;
        retimed_plan.start_state_ = plan.start_state_;


        std::vector<double> distance_between_points, desired_velocities,
                start(plan.trajectory_.joint_trajectory.joint_names.size(), 0),
                end(plan.trajectory_.joint_trajectory.joint_names.size(), 0),
                diff(plan.trajectory_.joint_trajectory.joint_names.size(), 0);
        std::vector<std::vector<double>> joint_differences;
        std::map<std::string, double> joint_velocites;

        // Find the cartesian distance between each joint trajectory point
        for (ulong i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {
            // If it's past the first point then set the joint position to the previous otherwise use the value from the
            // robot state as it should be the current /start state
            if (i > 0) {
                robot_state->setJointGroupPositions(joint_model_group,
                                                    plan.trajectory_.joint_trajectory.points[i - 1].positions);
            }
            robot_state->copyJointGroupPositions(joint_model_group, start);

            p_start = robot_state->getGlobalLinkTransform(
                    end_effector_link); // TODO: have this as an arg for the function, maybe have a default if robot_state has an ee link
            robot_state->setJointGroupPositions(joint_model_group,
                                                plan.trajectory_.joint_trajectory.points[i].positions);
            robot_state->copyJointGroupPositions(joint_model_group, end);

            p_end = robot_state->getGlobalLinkTransform(end_effector_link);
            distance_between_points.emplace_back((p_end.translation() - p_start.translation()).norm());

            // calculate the difference in the joint
            std::transform(end.begin(), end.end(), start.begin(), diff.begin(), std::minus<>());
            joint_differences.push_back(diff);
        }

        double current_desired_velocity, distance_from_start, distance_to_end, distance_acceleration, total_distance = 0;

        // Calculate the total distance travelled, this is to know when to decelerate .
        for (const auto &dist : distance_between_points) {
            total_distance += dist;
        }
        // Calculate the distance needed to accelerate and decelerate at the required value
        distance_acceleration = pow(desired_velocity, 2) / (2 * desired_acceleration);

        RCLCPP_INFO_STREAM(LOGGER, "Trajectory set to be retimed \n\tTotal distance: " << total_distance << \
                                   "\n\tAcceleration distance: " << distance_acceleration
        );

        if (2 * distance_acceleration > total_distance) {
            RCLCPP_WARN_STREAM(LOGGER, "The total distance of the trajectory is too low for the end-effector "
                                       "to get to full speed.");
        }

        distance_from_start = 0;

        for (ulong i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {

            distance_from_start += distance_between_points[i];
            distance_to_end = total_distance - distance_from_start;

            distance = distance_between_points[i];

            if (distance_from_start < distance_acceleration) {
                current_desired_velocity = sqrt(2 * desired_acceleration * distance_from_start);
                time_between_points = distance / current_desired_velocity;
                desired_velocities.emplace_back(current_desired_velocity);
            } else if (distance_to_end < distance_acceleration) {
                current_desired_velocity = sqrt(2 * desired_acceleration * distance_to_end);
                time_between_points = distance / current_desired_velocity;
                desired_velocities.emplace_back(current_desired_velocity);
            } else {
                time_between_points = distance / desired_velocity;
                desired_velocities.emplace_back(desired_velocity);
            }

            // Check for NANs or negative values or infinite
            if (isnan(time_between_points) || time_between_points < 0) {
                RCLCPP_WARN_STREAM(LOGGER,
                                   "Time between points given as " << time_between_points << " setting as zero.");
                time_between_points = 1e-5;
            } else if (isinf(time_between_points)) {
                RCLCPP_WARN_STREAM(LOGGER, "Time between points given as " << time_between_points
                                                                           << " setting as to 1mm/s or previous velocity if available.");
                // If we don't have anything to go on make it slow
                if (desired_velocities.empty()) {
                    time_between_points =
                            distance / 1e-3; // Set to 1mm/s, this should only really happen at the deceleration phase.
                }
                    // Otherwise lets use the last velocity, there should be enough interpolation between points that this isn't crazy
                else {
                    time_between_points = distance / desired_velocities[i - 1];
                }
            }

            // Calculate the new joint velocities proposed
            for (ulong j = 0; j < retimed_plan.trajectory_.joint_trajectory.joint_names.size(); j++) {
                joint_velocites[retimed_plan.trajectory_.joint_trajectory.joint_names[j]] =
                        joint_differences[i][j] / time_between_points;
            }
            // check joint limits against the velocity desired
            if (!trajectory_utilities::check_joint_velocities(robot_state->getRobotModel(), joint_velocites)) {
                RCLCPP_ERROR_STREAM(LOGGER, "Joint velocity limit violated, failed to retime trajectory");
                return false;
            }

            // Calculate the new time from start
            if (i > 0) {
                new_time_from_start =
                        rclcpp::Duration(retimed_plan.trajectory_.joint_trajectory.points[i - 1].time_from_start) + \
                rclcpp::Duration::from_seconds(time_between_points);
            } else {
                new_time_from_start = rclcpp::Duration::from_seconds(time_between_points);
            }

            RCLCPP_INFO_STREAM(LOGGER, "Point " << i << " of " << plan.trajectory_.joint_trajectory.points.size() - 1
                                                << "\nOriginal time from start: " << rclcpp::Duration(
                    retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start).seconds()
                                                << "\nDistance between points: " << distance
                                                << "\nRecalculated time from start: " << new_time_from_start.seconds()
                                                << std::endl);

            retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start = new_time_from_start;

        }
        return true;
    }
}
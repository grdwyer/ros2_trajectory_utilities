//
// Created by george on 1/4/22.
//

#include <ros2_trajectory_utilities/interpolation.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_utilities");
namespace trajectory_utilities {
    geometry_msgs::msg::Pose
    interpolate_between_pose(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, double factor) {
        // Split between translation and rotation(quaternion)
        Eigen::Translation<double, 3> t_start, t_end, t_interp;
        Eigen::Quaternion<double> q_start, q_end, q_interp;

        tf2::fromMsg(start.position, t_start.translation());
        tf2::fromMsg(end.position, t_end.translation());
        tf2::fromMsg(start.orientation, q_start);
        tf2::fromMsg(end.orientation, q_end);

        q_interp = q_start.slerp(factor, q_end);

        auto comp_start = t_start.translation() * Eigen::Scaling(1 - factor);
        auto comp_end = t_end.translation() * Eigen::Scaling(factor);
        t_interp = Eigen::Translation<double, 3>(comp_start + comp_end);

        geometry_msgs::msg::Pose p_interp;
        p_interp.orientation = tf2::toMsg(q_interp);
        p_interp.position = tf2::toMsg(t_interp.translation());
        return p_interp;
    }

    void interpolate_pose_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &original, double max_distance,
                                     double max_angle, std::vector<geometry_msgs::msg::PoseStamped> &interpolated) {
        double dist, angle;
        int interp_size = 10, dist_size, ang_size;
        geometry_msgs::msg::PoseStamped p_start, p_end, p_interp;
        for (ulong i = 0; i < original.size(); i++) {
            if (i < original.size() - 1) {
                p_start = original[i];
                p_end = original[i + 1];

                // determine the number of poses to interpolate by
                dist = trajectory_utilities::distance_between_poses(p_start.pose, p_end.pose);
                angle = trajectory_utilities::angular_distance_between_poses(p_start.pose, p_end.pose);

                dist_size = ceil(dist / max_distance);
                ang_size = ceil(angle / max_angle);
                interp_size = dist_size > ang_size ? dist_size : ang_size;

                for (int j = 0; j < interp_size; j++) {
                    p_interp.pose = interpolate_between_pose(p_start.pose, p_end.pose,
                                                             (double) j / (double) interp_size);
                    p_interp.header = p_start.header;
                    interpolated.push_back(p_interp);
                }
            } else {
                // Last pose just needs to be added to the list
                interpolated.push_back(original[i]);
            }
        }
    }

    void interpolate_pose_trajectory(std::vector<geometry_msgs::msg::Pose> &original, double max_distance,
                                     double max_angle, std::vector<geometry_msgs::msg::Pose> &interpolated) {
        double dist, angle;
        int interp_size = 10, dist_size, ang_size;
        geometry_msgs::msg::Pose p_start, p_end, p_interp;
        for (ulong i = 0; i < original.size(); i++) {
            if (i < original.size() - 1) {
                p_start = original[i];
                p_end = original[i + 1];

                // determine the number of poses to interpolate by
                dist = trajectory_utilities::distance_between_poses(p_start, p_end);
                angle = trajectory_utilities::angular_distance_between_poses(p_start, p_end);

                dist_size = ceil(dist / max_distance);
                ang_size = ceil(angle / max_angle);
                interp_size = dist_size > ang_size ? dist_size : ang_size;

                for (int j = 0; j < interp_size; j++) {
                    p_interp = interpolate_between_pose(p_start, p_end, (double) j / (double) interp_size);
                    interpolated.push_back(p_interp);
                }
            } else {
                // Last pose just needs to be added to the list
                interpolated.push_back(original[i]);
            }
        }
    }
}
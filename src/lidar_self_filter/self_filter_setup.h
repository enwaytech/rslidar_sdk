/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef RSLIDAR_SDK_SELF_FILTER_SETUP_H_
#define RSLIDAR_SDK_SELF_FILTER_SETUP_H_

#include <lidar_self_filter/lidar_self_filter_setup.h>
#include <lidar_self_filter/lidar_settings.h>

#include "msg/ros_msg_translator.h"

#include <safety_utils/utils_params.h>

#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <optional>
#include <string>
#include <tuple>

namespace robosense
{
  namespace lidar
  {
    class SelfFilterSetup
    {
    public:
      explicit SelfFilterSetup(ros::NodeHandle& node_handle,
                              lidar_self_filter::LidarSettings lidar_settings,
                              std::string sensor_frame,
                              std::string save_file_path);

      void filter(const LidarPointCloudMsg& msg);

    private:
      [[nodiscard]] bool inBufferedFootprint(const PointT& laser_return);

      constexpr static double k_footprint_buffer_x_width_ = 0.035;
      constexpr static double k_footprint_buffer_y_width_ = 0.075;
      constexpr static double k_min_z_ = 0.05;

      const safety_utils::UtilsParams robot_params_;
      const lidar_self_filter::LidarSettings lidar_settings_;

      lidar_self_filter::LidarSelfFilterSetup lidar_self_filter_;

      const std::string sensor_frame_;
      tf2_ros::Buffer transform_buffer_;
      tf2_ros::TransformListener transform_listener_;

      ros::Publisher pub_;
      ros::Publisher pub_self_filter_markers_;
      visualization_msgs::Marker visualization_points_;
    };

  } // namespace lidar
} // namespace robosense


#endif
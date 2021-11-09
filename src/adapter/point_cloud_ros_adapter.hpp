/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#ifdef ROS_FOUND
#include <ros/ros.h>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"

#include <optional>
#include "lidar_self_filter/self_filter_setup.h"
#include "lidar_self_filter/filter.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robosense
{
namespace lidar
{
class PointCloudRosAdapter : virtual public AdapterBase
{
public:
  PointCloudRosAdapter() = default;
  ~PointCloudRosAdapter() = default;
  void init(const YAML::Node& config);
  void sendPointCloud(const LidarPointCloudMsg& msg);


private:
  lidar_self_filter::LidarSettings makeSelfFilterSettings(const YAML::Node& config) const;
  std::optional<geometry_msgs::TransformStamped> lookupTransformToBaseLink() const;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher point_cloud_pub_;
  std::optional<SelfFilterSetup> self_filter_setup_;
  std::optional<lidar_self_filter::Filter> self_filter_;
  bool self_filter_activated_;
  bool self_filter_setup_activ_;
  tf2_ros::Buffer transform_buffer_;
  std::optional<tf2_ros::TransformListener> transform_listener_;
  std::string frame_id_;
};

inline std::optional<geometry_msgs::TransformStamped>
PointCloudRosAdapter::lookupTransformToBaseLink() const
{
  constexpr std::string_view k_base_link_frame {"base_link"};
  try
  {
    return transform_buffer_.lookupTransform("base_link", frame_id_, ros::Time(0.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Failed transform of point from sensor frame " << frame_id_ << " to base_link: " << ex.what()
                                                                   << ". Self-filtering will not be possible");
    return std::nullopt;
  }
}

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  tf2_ros::TransformListener transform_listener(transform_buffer_);
  //transform_listener_.emplace(transform_listener);

  bool send_point_cloud_ros;
  std::string ros_send_topic;
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  yamlRead<bool>(config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");
  RS_WARNING << "ROS_SEND_TOPIC: " << ros_send_topic << RS_REND;
  if (send_point_cloud_ros)
  {
    point_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
  }

  const lidar_self_filter::LidarSettings self_filter_lidar_settings = makeSelfFilterSettings(config);
  //YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  //YAML::Node driver_config = yamlSubNodeAbort(lidar_config, "driver");
  std::string frame_id;
  yamlRead<std::string>(config["driver"], "frame_id", frame_id, "rslidar");
  RS_WARNING << "Frame ID: " << frame_id << RS_REND;
  frame_id_ = frame_id;
  const std::string filter_file_path {"config/self_filter_data_" + frame_id};


  bool self_filter_setup;
  yamlRead<bool>(config, "self_filter_setup", self_filter_setup, true);
  self_filter_setup_activ_ = self_filter_setup;
  if (self_filter_setup)
  {
    ros::NodeHandle filter_handle {"~/lidar_self_filter"};
    self_filter_setup_.emplace(filter_handle, self_filter_lidar_settings, frame_id, filter_file_path);
  }

  yamlRead<bool>(config, "self_filter_activated", self_filter_activated_, false);
  if (self_filter_activated_ && !self_filter_setup)
  {
    const lidar_self_filter::Filter self_filter {
        filter_file_path, self_filter_lidar_settings, lookupTransformToBaseLink()};
    self_filter_.emplace(self_filter);
  }
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  RS_WARNING << "PointCloudRosAdapter: Point Cloud size: " << msg.point_cloud_ptr->size() << RS_REND;

  if (self_filter_setup_activ_)
  {
    RS_WARNING << frame_id_ << ": SELF FILTER SETUP ACTIVE" << RS_REND;
    self_filter_setup_->filter(msg);
  }

  if (self_filter_activated_)
  {
    {
    // TODO: Check each point if self filter point
    // Assemble / remove point from pointcloud
    // publish self filtered point cloud
    }
  }
  point_cloud_pub_.publish(toRosMsg(msg));
}

inline lidar_self_filter::LidarSettings
PointCloudRosAdapter::makeSelfFilterSettings(const YAML::Node& config) const
{

  //YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar3");
  //YAML::Node driver_config = yamlSubNodeAbort(lidar_config, "driver");
  std::string frame_id {"rslidar"};
  yamlRead<std::string>(config["driver"], "frame_id", frame_id, "rslidar");

  lidar_self_filter::LidarSettings settings;
  settings.serial = frame_id;
  settings.link = frame_id;

  //constexpr double deg_resolution_per_hz = 0.24; // Honeycomb Laser Bear 1 specific
  //const double scans_in_fov = config_->fieldOfViewDeg() / (config_->spinFrequency() * deg_resolution_per_hz);
  settings.yaw_resolution = 1.00;//config_->fieldOfViewDeg() / scans_in_fov;

  //constexpr double half = 0.5;

  settings.min_yaw = 0.0; //config_->fieldOfViewDirectionDeg() - (half * config_->fieldOfViewDeg());
  settings.max_yaw = 360.0; //config_->fieldOfViewDirectionDeg() + (half * config_->fieldOfViewDeg());

  // fixing pitch resolution 0.5x highest resolution to cope with all pitch tables
  //constexpr double min_pitch_resolution {0.75};
  settings.pitch_resolution = 1.00;//min_pitch_resolution;

  // The alignment of the pitch angles to the spacing in the filter should be kept
  // so we do padding with a multiple of the resolution
  //constexpr double max_pitch_deviation_degree = 0.5;
  //const double pitch_padding_size = std::ceil(max_pitch_deviation_degree / settings.pitch_resolution);
  //const double padding = pitch_padding_size * settings.pitch_resolution;

  settings.min_pitch = 0.0;  //config_->pitchTable().back() - padding;
  settings.max_pitch = 90.0; //config_->pitchTable().front() + padding;

  return settings;
}

}  // namespace lidar
}  // namespace robosense
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
namespace robosense
{
namespace lidar
{
class PointCloudRosAdapter : virtual public AdapterBase
{
public:
  PointCloudRosAdapter() = default;
  ~PointCloudRosAdapter() = default;
  void init(const YAML::Node& config);
  void sendPointCloud(const LidarPointCloudMsg& msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  bool send_point_cloud_ros;
  std::string ros_send_topic;
  node_ptr_.reset(new rclcpp::Node("rslidar_points_adapter"));
  yamlRead<bool>(config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");
  if (send_point_cloud_ros)
  {
    point_cloud_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, 1);
  }
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_pub_->publish(toRosMsg(msg));
}

}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND

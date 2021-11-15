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
#include <pcl/filters/extract_indices.h>
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
  bool self_filter_setup_active_;
  bool self_filter_enabled_;
  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_{transform_buffer_};
  std::string frame_id_;
};

inline std::optional<geometry_msgs::TransformStamped>
PointCloudRosAdapter::lookupTransformToBaseLink() const
{
  constexpr std::string_view k_base_link_frame {"base_link"};
  try
  {
    return transform_buffer_.lookupTransform(static_cast<std::string>(k_base_link_frame), frame_id_, ros::Time(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Failed transform of point from sensor frame " << frame_id_ << " to base_link: " << ex.what()
                                                                   << ". Self-filtering will not be possible");
    geometry_msgs::TransformStamped test;
    test.child_frame_id = frame_id_;
    test.header.frame_id = "base_link";
    test.transform.translation.x = 0.0;
    test.transform.translation.z = 0.0;
    test.transform.rotation.w = 1.0;
    return test; //std::nullopt;
  }
}

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  //tf2_ros::TransformListener transform_listener(transform_buffer_);
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

  std::string frame_id;
  yamlRead<std::string>(config["driver"], "frame_id", frame_id, "rslidar");
  RS_WARNING << "Frame ID: " << frame_id << RS_REND;
  frame_id_ = frame_id;
  const std::string filter_file_path {"config/self_filter_data_" + frame_id};


  bool self_filter_setup;
  yamlRead<bool>(config["driver"], "self_filter_setup", self_filter_setup, false);
  self_filter_setup_active_ = self_filter_setup;
  if (self_filter_setup)
  {
    RS_WARNING << "SELF FILTER SETUP ENABLED" << RS_REND;
    ros::NodeHandle filter_handle {"~/lidar_self_filter"};
    self_filter_setup_.emplace(filter_handle, self_filter_lidar_settings, frame_id, filter_file_path);
  }

  yamlRead<bool>(config["driver"], "self_filter_enabled", self_filter_enabled_, false);
  if (self_filter_enabled_ && !self_filter_setup)
  {
  RS_WARNING << "SELF FILTER ENABLED" << RS_REND;
    const lidar_self_filter::Filter self_filter {
        filter_file_path, self_filter_lidar_settings, lookupTransformToBaseLink()};
    self_filter_.emplace(self_filter);
  }
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  RS_WARNING << "PointCloudRosAdapter: Point Cloud size: " << msg.point_cloud_ptr->size() << RS_REND;

  if (self_filter_setup_active_)
  {
    RS_WARNING << frame_id_ << ": SELF FILTER SETUP ACTIVE" << RS_REND;
    self_filter_setup_->filter(msg);
  }
  std::vector<float> pitch;
  if (self_filter_enabled_)
  {
    RS_WARNING << "Self Filter activated" << RS_REND;
    unsigned int number_of_filtered_points {0};

    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
    float old_yaw {0.0F};
    float old_pitch {0.0F};
    for (size_t idx = 0; idx < msg.point_cloud_ptr->size(); ++idx)
    {
      const PointT& point {msg.point_cloud_ptr->points[idx]};
      if (std::isnan(point.x)
          || std::isnan(point.y)
          || std::isnan(point.z)
          || std::isnan(point.yaw)
          || std::isnan(point.pitch)
          || std::isnan(point.range))
      {
        continue;
      }
#ifdef DEBUG
      if (point.pitch != old_pitch)
      { 
          old_pitch = point.pitch;
          pitch.push_back(point.pitch);
          //RS_WARNING << "Pitch: " << point.pitch << RS_REND;
      }
      if (point.yaw != old_yaw)
      { 
          old_yaw = point.yaw;
          RS_WARNING << " Yaw: " << point.yaw << RS_REND;
      }
#endif
      if (!self_filter_->isSelfPoint(point.yaw,
                                     point.pitch,
                                     point.range,
                                     point.x,
                                     point.y,
                                     point.z))
      {
        filtered_cloud->push_back(point);
      }
      else
      {
        number_of_filtered_points++;
      }
    }
#ifdef DEBUG
    std::sort(pitch.begin(), pitch.end());
    auto last = std::unique(pitch.begin(), pitch.end());
    pitch.erase(last, pitch.end());
    float last_pitch {0.0F};
    std::vector<float> diffs;
    for (auto p : pitch)
    {
      RS_WARNING << "Pitch: " << std::setprecision(10) << p << " diff: " << p - last_pitch << RS_REND;
      last_pitch = p;
      diffs.push_back(p - last_pitch);
    }
    float sum = std::accumulate(diffs.begin()+1, diffs.end(), 0.0);
    float average_diff = sum / (diffs.size() - 1);
    RS_WARNING << "Average Diff: " << average_diff << RS_REND;
#endif
    filtered_cloud->header = msg.point_cloud_ptr->header;
    RS_WARNING << "Filtered cloud: " << filtered_cloud->points.size() << RS_REND;

    LidarPointCloudMsg filtered_msg{filtered_cloud};
    filtered_msg.timestamp = msg.timestamp;
    filtered_msg.seq = msg.seq;
    filtered_msg.frame_id = msg.frame_id;

    RS_WARNING << frame_id_ << ": Filtered points: " << number_of_filtered_points << RS_REND;
    RS_WARNING << "Output cloud" << filtered_msg.point_cloud_ptr->size() << RS_REND;
    point_cloud_pub_.publish(toRosMsg(filtered_msg));
    // TODO: Check each point if self filter point
    // Assemble / remove point from pointcloud
    // publish self filtered point cloud
    return;
  }
  point_cloud_pub_.publish(toRosMsg(msg));
}

inline lidar_self_filter::LidarSettings
PointCloudRosAdapter::makeSelfFilterSettings(const YAML::Node& config) const
{
  std::string frame_id {"rslidar"};
  yamlRead<std::string>(config["driver"], "frame_id", frame_id, "rslidar");

  RS_WARNING << "makeSelfFilterSettings: " << frame_id << RS_REND;

  lidar_self_filter::LidarSettings settings;
  settings.serial = frame_id;
  settings.link = frame_id;

  //constexpr double deg_resolution_per_hz = 0.24; // Honeycomb Laser Bear 1 specific
  //const double scans_in_fov = config_->fieldOfViewDeg() / (config_->spinFrequency() * deg_resolution_per_hz);
  settings.yaw_resolution = 0.01;

  //constexpr double half = 0.5;

  settings.min_yaw = 0.0; //config_->fieldOfViewDirectionDeg() - (half * config_->fieldOfViewDeg());
  settings.max_yaw = 360.0; //config_->fieldOfViewDirectionDeg() + (half * config_->fieldOfViewDeg());

  // fixing pitch resolution 0.5x highest resolution to cope with all pitch tables
  //constexpr double min_pitch_resolution {0.75};
  settings.pitch_resolution = 2.81;//min_pitch_resolution;

  // The alignment of the pitch angles to the spacing in the filter should be kept
  // so we do padding with a multiple of the resolution
  //constexpr double max_pitch_deviation_degree = 0.5;
  //const double pitch_padding_size = std::ceil(max_pitch_deviation_degree / settings.pitch_resolution);
  //const double padding = pitch_padding_size * settings.pitch_resolution;

  settings.min_pitch = 2.31;  //config_->pitchTable().back() - padding;
  settings.max_pitch = 89.5; //config_->pitchTable().front() + padding;

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

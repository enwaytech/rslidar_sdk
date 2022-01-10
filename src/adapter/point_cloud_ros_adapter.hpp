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

#include "dust_filter_robosense/dust_filter.h"
#include "lidar_self_filter/filter.h"
#include "lidar_self_filter/self_filter_setup.h"

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

  bool send_point_cloud_ros_unfiltered_;
  ros::Publisher point_cloud_pub_unfiltered_;

  bool send_point_cloud_ros_self_filtered_;
  ros::Publisher point_cloud_pub_self_filtered_;

  bool send_point_cloud_ros_;
  ros::Publisher point_cloud_pub_;
  bool remove_duplicates_;
  std::string frame_id_;
  
  bool self_filter_setup_enabled_;
  std::optional<SelfFilterSetup> self_filter_setup_;
  
  bool self_filter_enabled_;
  std::optional<lidar_self_filter::Filter> self_filter_;

  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_{transform_buffer_};

  bool dust_filter_enabled_;
  dust_filter_robosense::DustFilter<PointT> dust_filter_;
};

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  usleep(10000);

  std::string ros_send_topic;
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  yamlRead<bool>(config, "send_point_cloud_ros", send_point_cloud_ros_, false);
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");

  if (send_point_cloud_ros_)
  {
    point_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
  }

  yamlRead<bool>(config["ros"], "send_point_cloud_ros_unfiltered", send_point_cloud_ros_unfiltered_, false);

  if (send_point_cloud_ros_unfiltered_)
  {
    point_cloud_pub_unfiltered_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic + "_unfiltered", 10);
  }

  yamlRead<bool>(config["ros"], "send_point_cloud_ros_self_filtered", send_point_cloud_ros_self_filtered_, false);

  if (send_point_cloud_ros_self_filtered_)
  {
    point_cloud_pub_self_filtered_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic + "_self_filtered", 10);
  }

  yamlRead<bool>(config["ros"], "remove_duplicates", remove_duplicates_, false);
  yamlRead<std::string>(config["driver"], "frame_id", frame_id_, "rslidar");

  yamlRead<bool>(config["self_filter"], "self_filter_setup", self_filter_setup_enabled_, false);
  if (self_filter_setup_enabled_)
  {
    ros::NodeHandle filter_handle {"~/lidar_self_filter_" + frame_id_}; // _" + frame_id_};
    const lidar_self_filter::LidarSettings self_filter_lidar_settings = makeSelfFilterSettings(config);
    const std::string filter_file_path {"config/self_filter_data_" + frame_id_};
    self_filter_setup_.emplace(filter_handle, self_filter_lidar_settings, frame_id_, filter_file_path);
  }
  
  yamlRead<bool>(config["self_filter"], "self_filter_enabled", self_filter_enabled_, false);
  if(self_filter_enabled_ && !self_filter_setup_enabled_)
  {
    const lidar_self_filter::LidarSettings self_filter_lidar_settings = makeSelfFilterSettings(config);
    const std::string filter_file_path {"config/self_filter_data_" + frame_id_};
    const lidar_self_filter::Filter self_filter {
        filter_file_path, self_filter_lidar_settings, lookupTransformToBaseLink()};
    self_filter_.emplace(self_filter);
  }

  yamlRead<bool>(config["ros"], "dust_filter_enabled", dust_filter_enabled_, false);
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{

  if (self_filter_setup_enabled_)
  {
#ifdef POINT_TYPE_XYZRPYINR
    self_filter_setup_->filter(msg);
#else
    RS_WARNING << "Self filter setup only works with POINT_TYPE_XYZRPYINR" << RS_REND;
    exit(1);
#endif
    return;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  cloud->header = msg.point_cloud_ptr->header;
  pcl::copyPointCloud(*(msg.point_cloud_ptr), *cloud);

  if (remove_duplicates_)
  {
    pcl::PointIndices indices;
    pcl::PointIndices::Ptr duplicates (new pcl::PointIndices ());
    //RS_WARNING << "Size before removing duplicates: " << cloud->points.size();
    // TODO: Get channels per block
    constexpr unsigned int channels_per_block = 32;

    unsigned int blks_cloud = cloud->points.size() / channels_per_block;
    for (unsigned int blk_idx = 0; blk_idx < blks_cloud; blk_idx = blk_idx + 2)
    {
      for (unsigned int channel_idx = 0; channel_idx < channels_per_block; channel_idx++)
      {
        const int idx = blk_idx * channels_per_block + channel_idx;
        const PointT& first_return {cloud->points[idx]};
        const PointT& second_return {cloud->points[idx + channels_per_block]};
        if (first_return.yaw == second_return.yaw && first_return.pitch == second_return.pitch)
        {
          if (first_return.range == second_return.range)
          {
            indices.indices.emplace_back(idx);
            continue;
          }
        }

        indices.indices.emplace_back(idx);
        indices.indices.emplace_back(idx + channels_per_block);
      }
    }

    pcl::copyPointCloud(*(cloud), indices.indices, *cloud);
    //RS_WARNING << " after: " << cloud->points.size() << RS_REND;
  }
  if (send_point_cloud_ros_unfiltered_)
  {
    LidarPointCloudMsg unfiltered_msg{cloud};
    unfiltered_msg.timestamp = msg.timestamp;
    unfiltered_msg.seq = msg.seq;
    unfiltered_msg.frame_id = msg.frame_id;
    point_cloud_pub_unfiltered_.publish(toRosMsg(unfiltered_msg));
  }


  if (self_filter_enabled_)
  {
#ifdef POINT_TYPE_XYZRPYINR
    pcl::PointIndices indices;
    //RS_WARNING << "Size before self-filtering: " << cloud->points.size();
    for (size_t idx = 0; idx < cloud->points.size(); ++idx)
    {
      const PointT& point {cloud->points[idx]};
      if (std::isnan(point.x)
          || std::isnan(point.y)
          || std::isnan(point.z)
          || std::isnan(point.yaw)
          || std::isnan(point.pitch)
          || std::isnan(point.range))
      {
        continue;
      }
      if (!self_filter_->isSelfPoint(point.yaw,
                                     point.pitch,
                                     point.range,
                                     point.x,
                                     point.y,
                                     point.z))
      {
        indices.indices.emplace_back(idx);
      }
    }
    pcl::copyPointCloud(*cloud, indices.indices, *cloud);
    //RS_WARNING << " after: " << cloud->points.size() << RS_REND;
#else
    RS_WARNING << "Self filter only works with POINT_TYPE_XYZRPYINR" << RS_REND;
    exit(1);
#endif
  }

  if (send_point_cloud_ros_self_filtered_)
  {
    LidarPointCloudMsg self_filtered_msg{cloud};
    self_filtered_msg.timestamp = msg.timestamp;
    self_filtered_msg.seq = msg.seq;
    self_filtered_msg.frame_id = msg.frame_id;
    point_cloud_pub_self_filtered_.publish(toRosMsg(self_filtered_msg));
  }

  if (dust_filter_enabled_)
  {
    //RS_WARNING << "Size before dust-filtering: " << cloud->points.size();
    dust_filter_.startNewPointCloud(cloud->header, cloud->size());
    for (const auto p : *cloud)
    {
      dust_filter_.addMeasurement(p);
    }
    *cloud = dust_filter_.getFilteredPointCloud();
    //RS_WARNING << " after: " << cloud->points.size() << RS_REND;
  }

  if (send_point_cloud_ros_)
  {
    LidarPointCloudMsg filtered_msg{cloud};
    filtered_msg.timestamp = msg.timestamp;
    filtered_msg.seq = msg.seq;
    filtered_msg.frame_id = msg.frame_id;
    point_cloud_pub_.publish(toRosMsg(filtered_msg));
  }
}

inline lidar_self_filter::LidarSettings
PointCloudRosAdapter::makeSelfFilterSettings(const YAML::Node& config) const
{
  std::string frame_id {"rslidar"};
  yamlRead<std::string>(config["driver"], "frame_id", frame_id, "rslidar");

  lidar_self_filter::LidarSettings settings;
  settings.serial = frame_id;
  settings.link = frame_id;

  double yaw_resolution;
  yamlRead<double>(config["self_filter"], "yaw_resolution", yaw_resolution, 0.01);
  settings.yaw_resolution = yaw_resolution;

  double min_yaw;
  yamlRead<double>(config["self_filter"], "min_yaw", min_yaw, 0.0);
  settings.min_yaw = min_yaw;

  double max_yaw;
  yamlRead<double>(config["self_filter"], "max_yaw", max_yaw, 360.0);
  settings.max_yaw = max_yaw;

  double pitch_resolution;
  yamlRead<double>(config["self_filter"], "pitch_resolution", pitch_resolution, 2.81);
  settings.pitch_resolution = pitch_resolution;

  double min_pitch;
  yamlRead<double>(config["self_filter"], "min_pitch", min_pitch, 2.31);
  settings.min_pitch = min_pitch;

  double max_pitch;
  yamlRead<double>(config["self_filter"], "max_pitch", max_pitch, 89.5);
  settings.max_pitch = max_pitch;

  return settings;
}

inline std::optional<geometry_msgs::TransformStamped>
PointCloudRosAdapter::lookupTransformToBaseLink() const
{
  constexpr std::string_view k_base_link_frame {"base_link"};

  constexpr double transform_timeout {2.0};

  try
  {
    if (transform_buffer_.canTransform(static_cast<std::string>(k_base_link_frame), frame_id_, ros::Time(0.0), ros::Duration(transform_timeout)))
    {
      return transform_buffer_.lookupTransform(static_cast<std::string>(k_base_link_frame), frame_id_, ros::Time(0.0));
    }
    ROS_WARN_STREAM("Failed transform of point from sensor frame " << frame_id_ << " to base_link: " << ". Self-filtering will not be possible");
    return std::nullopt;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Failed transform of point from sensor frame " << frame_id_ << " to base_link: " << ex.what()
                                                                  << ". Self-filtering will not be possible");
#ifdef DEBUG
    geometry_msgs::TransformStamped test;
    test.child_frame_id = frame_id_;
    test.header.frame_id = "base_link";
    test.transform.translation.x = 0.0;
    test.transform.translation.z = 0.0;
    test.transform.rotation.w = 1.0;
#endif
    return std::nullopt;
  }
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
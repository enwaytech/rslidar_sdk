/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

//#include "honeycomb_ros/self_filter_setup.h"
//#include "honeycomb_ros/helpers_waymo.h"
#include "lidar_self_filter/self_filter_setup.h"

//#include <dust_filter/dust_filter.h>
//#include <dust_filter/params.h>

#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <limits>
#include <string_view>
#include <utility>

constexpr std::string_view k_base_link_frame {"base_link"};

robosense::lidar::SelfFilterSetup::SelfFilterSetup(ros::NodeHandle& node_handle,
                                                lidar_self_filter::LidarSettings lidar_settings,
                                                std::string sensor_frame,
                                                std::string save_file_path)
  //: robot_params_ {}
  : lidar_settings_ {std::move(lidar_settings)}
  , lidar_self_filter_ {node_handle, lidar_settings_, save_file_path}
  , sensor_frame_ {std::move(sensor_frame)}
  , transform_listener_ {transform_buffer_}
{
  visualization_points_.header.frame_id = static_cast<std::string>(k_base_link_frame);
  visualization_points_.type = visualization_msgs::Marker::POINTS;
  visualization_points_.color.a = 1.0F;
  visualization_points_.pose.orientation.w = 1.0;
  constexpr double point_marker_size = 0.02;
  visualization_points_.scale.x = point_marker_size;
  visualization_points_.scale.y = point_marker_size;

  pub_ = node_handle.advertise<visualization_msgs::Marker>("points_transformed", 1);
}

void
robosense::lidar::SelfFilterSetup::filter(const LidarPointCloudMsg& msg)
{
  visualization_points_.points.clear();

  if (!lidar_self_filter_.startNewScan())
  {
    return;
  }

  for (const auto& point : *(msg.point_cloud_ptr))
  {
    if (std::isnan(point.range) || std::isnan(point.yaw) || std::isnan(point.pitch))
    {
      continue;
    }

    lidar_self_filter_.insertReading(static_cast<double>(point.yaw), static_cast<double>(point.pitch), point.range);
  }

  #if 0
  for (const auto& shot : scans.GetShots())
  {
    const int num_returns = shot.NumReturns();

    if (num_returns == 0)
    {
      continue;
    }

    const double pitch_degrees = shot.GetBeamAnglesInDegrees().pitch_deg;
    const double yaw_degrees = shot.GetBeamAnglesInDegrees().yaw_deg;

    double min_range = std::numeric_limits<double>::infinity();
    bool has_return_inside_footprint = false;
    for (int return_index = 0; return_index < num_returns; ++return_index)
    {
      const waymo::Return& ret = shot.GetReturn(return_index);

      if (dust_filter::isCertainDust(dust_filter::CertainDustParams(),
                                     shot.NumReturns(),
                                     ret.Index(),
                                     static_cast<float>(ret.Intensity()),
                                     static_cast<float>(ret.PulseWidthInMeters())))
      {
        continue;
      }

      if (ret.RangeInMeters() < waymo_helpers::g_min_range)
      {
        continue;
      }

      if (inBufferedFootprint(ret))
      {
        if (!has_return_inside_footprint || ret.RangeInMeters() < min_range)
        {
          // set min. range if no return inside footprint yet OR closer point (inside footprint)
          min_range = ret.RangeInMeters();
        }

        has_return_inside_footprint = true;
      }
      else if (!has_return_inside_footprint)
      {
        // update with returns outside of footprint only if we didn't get any return inside footprint yet
        if (ret.RangeInMeters() < min_range)
        {
          min_range = ret.RangeInMeters();
        }
      }
    }

    if (std::isinf(min_range) || !has_return_inside_footprint)
    {
      continue;
    }

    lidar_self_filter_.insertReading(yaw_degrees, pitch_degrees, static_cast<float>(min_range));
  }
#endif
  if (pub_.getNumSubscribers() > 0)
  {
    visualization_points_.header.stamp = ros::Time::now();
    pub_.publish(visualization_points_);
  }
}
#if 0
bool
robosense::lidar::SelfFilterSetup::inBufferedFootprint(const waymo::Return& laser_return)
{

  const waymo::Vec3d cartesian_coordinates {laser_return.GetCoordinates()};
  geometry_msgs::PointStamped sensor_point;
  sensor_point.header.frame_id = sensor_frame_;
  sensor_point.point.x = cartesian_coordinates.x;
  sensor_point.point.y = cartesian_coordinates.y;
  sensor_point.point.z = cartesian_coordinates.z;

  geometry_msgs::PointStamped transformed_point;
  try
  {
    transform_buffer_.transform(sensor_point, transformed_point, static_cast<std::string>(k_base_link_frame));
  }
  catch (tf2::TransformException& ex)
  {
    const std::string warning {"Failed transform of point from sensor frame " + sensor_frame_ + " to "
                               + static_cast<std::string>(k_base_link_frame) + ": " + ex.what()};
    constexpr double throttle_secs = 5.0;
    ROS_WARN_STREAM_THROTTLE(throttle_secs, warning);
    return false;
  }

  const double max_x = robot_params_.baseLinkFrontOffset() + k_footprint_buffer_width_;
  const double min_x = robot_params_.baseLinkBackOffset() - k_footprint_buffer_width_;
  const double max_y = (robot_params_.footprintWidth() * 0.5) + k_footprint_buffer_width_;

  const geometry_msgs::Point& base_link_point = transformed_point.point;

  if (base_link_point.z >= k_min_z_ && base_link_point.x >= min_x && base_link_point.x <= max_x
      && std::abs(base_link_point.y) <= max_y)
  {
    visualization_points_.points.push_back(base_link_point);
    return true;
  }
  return false;
}
#endif
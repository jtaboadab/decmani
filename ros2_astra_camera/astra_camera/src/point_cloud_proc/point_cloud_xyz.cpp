// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include "astra_camera/point_cloud_proc/point_cloud_xyz.h"

namespace astra_camera {

PointCloudXyzNode::PointCloudXyzNode(const rclcpp::NodeOptions &options)
    : Node("PointCloudXyzNode", options) {
  // Read parameters
  queue_size_ = static_cast<int>(declare_parameter<int>("queue_size", 5));

  // Monitor whether anyone is subscribed to the output
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNode::connectCb, this);
  connectCb();

  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  pub_point_cloud_ = create_publisher<PointCloud2>("depth/points", rclcpp::QoS{1});
}

template <typename T>
void PointCloudXyzNode::convertDepth(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                                     sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg,
                                     const image_geometry::PinholeCameraModel &model,
                                     double range_max) {
  // Use correct principal point from calibration
  auto center_x = static_cast<float>(model.cx());
  auto center_y = static_cast<float>(model.cy());

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters(T(1));
  auto constant_x = static_cast<float>(unit_scaling / model.fx());
  auto constant_y = static_cast<float>(unit_scaling / model.fy());
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth)) {
        if (range_max != 0.0) {
          depth = DepthTraits<T>::fromMeters(range_max);
        } else {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (static_cast<float>(u) - center_x) * depth * constant_x;
      *iter_y = (static_cast<float>(v) - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzNode::connectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!sub_depth_) {
    auto custom_qos = rmw_qos_profile_sensor_data;
    custom_qos.depth = queue_size_;

    sub_depth_ = image_transport::create_camera_subscription(
        this, "depth/image_raw",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) { depthCb(msg, info); },
        "raw", custom_qos);
  }
}

void PointCloudXyzNode::depthCb(const Image::ConstSharedPtr &depth_msg,
                                const CameraInfo::ConstSharedPtr &info_msg) {
  auto cloud_msg = std::make_shared<PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  // Convert Depth Image to Pointcloud
  if (depth_msg->encoding == enc::TYPE_16UC1) {
    convertDepth<uint16_t>(depth_msg, cloud_msg, model_);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    convertDepth<float>(depth_msg, cloud_msg, model_);
  } else {
    RCLCPP_ERROR(logger_, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  pub_point_cloud_->publish(*cloud_msg);
}

}  // namespace astra_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::PointCloudXyzNode)

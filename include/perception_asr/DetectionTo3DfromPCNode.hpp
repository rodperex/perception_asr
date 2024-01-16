// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PERCEPTION_ASR__DETECTIONTO3DFROMPCNODE_HPP_
#define PERCEPTION_ASR__DETECTIONTO3DFROMPCNODE_HPP_

#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace perception_asr
{

class DetectionTo3DfromPCNode : public rclcpp::Node
{
public:
  DetectionTo3DfromPCNode();

private:
  void callback_sync(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg);
  void callback_info(sensor_msgs::msg::CameraInfo::UniquePtr msg);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray> MySyncPolicy;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc2_sub_;
  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg_;
  

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
};

}  // namespace perception_asr

#endif  // PERCEPTION_ASR__DETECTIONTO3DFROMPCNODE_HPP_

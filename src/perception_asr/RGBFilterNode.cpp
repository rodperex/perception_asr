#include <memory>

#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

#include "perception_asr/RGBFilterNode.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_asr
{

using std::placeholders::_1;

RGBFilterNode::RGBFilterNode()
: Node("rgb_filter_node"), r_(0), R_(255), g_(0), G_(255), b_(0), B_(255)
{
  image_sub_ = image_transport::create_subscription(
    this, "input_image", std::bind(&RGBFilterNode::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  image_pub_ = image_transport::create_publisher(
    this, "output_image", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "output_detection_2d", rclcpp::SensorDataQoS().reliable());

  // Trackbars for adjusting RGB filtering parameters
  cv::namedWindow("Filtered Image");
  cv::createTrackbar("Low R", "Filtered Image", &r_, 255);
  cv::createTrackbar("High R", "Filtered Image", &R_, 255);
  cv::createTrackbar("Low G", "Filtered Image", &g_, 255);
  cv::createTrackbar("High G", "Filtered Image", &G_, 255);
  cv::createTrackbar("Low B", "Filtered Image", &b_, 255);
  cv::createTrackbar("High B", "Filtered Image", &B_, 255);
}

void RGBFilterNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Convert sensor_msgs::msg::Image -> cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Perform RGB filtering
  cv::Mat1b filtered;
  cv::inRange(cv_ptr->image, cv::Scalar(b_, g_, r_), cv::Scalar(B_, G_, R_), filtered);

  // Publish the filtered image
  cv_bridge::CvImage out_image;
  out_image.header = msg->header;
  out_image.encoding = sensor_msgs::image_encodings::BGR8;
  cv_ptr->image.copyTo(out_image.image, filtered);

  // Display the filtered image
  cv::imshow("Filtered Image", out_image.image);
  cv::waitKey(30);

  if (image_pub_.getNumSubscribers() > 0) {
    image_pub_.publish(out_image.toImageMsg());
  }

  // Perform 2D object detection
  if (detection_pub_->get_subscription_count() > 0) {
    cv::Rect bbx = cv::boundingRect(filtered);

    auto m = cv::moments(filtered, true);
    if (m.m00 < 0.000001) {return;}
    int cx = m.m10 / m.m00;
    int cy = m.m01 / m.m00;

    vision_msgs::msg::Detection2D detection_msg;
    detection_msg.header = msg->header;
    detection_msg.bbox.center.position.x = cx;
    detection_msg.bbox.center.position.y = cy;
    detection_msg.bbox.size_x = bbx.width;
    detection_msg.bbox.size_y = bbx.height;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis_msg;
    hypothesis_msg.hypothesis.class_id = "ND";
    hypothesis_msg.hypothesis.score = 1.0;
    detection_msg.results.push_back(hypothesis_msg);  


    vision_msgs::msg::Detection2DArray detection_array_msg;
    detection_array_msg.header = msg->header;
    detection_array_msg.detections.push_back(detection_msg);

    detection_pub_->publish(detection_array_msg);
  }

  
}

}  // namespace perception_asr

#ifndef APRILTAG_ROS_DETECTOR_NODE_H_
#define APRILTAG_ROS_DETECTOR_NODE_H_

#include <mutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <AprilTags/TagDetector.h>

namespace aprilslam {

class DetectorNode {
 public:
  DetectorNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);

  ros::NodeHandle nh_;
  std::string tag_family_;
  double tag_size_;
  bool cam_calibrated_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  ros::Publisher pub_detections_;
  std::mutex connect_mutex_;
  image_geometry::PinholeCameraModel model_;
  cv::Mat gamma_lookup_;
  AprilTags::TagDetector tag_detector_;
};

}  // namespace aprilslam

#endif  // APRILSLAM_DETECTOR_NODE_H_

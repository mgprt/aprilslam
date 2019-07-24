#include "aprilslam/detector_node.h"
#include "aprilslam/utils.h"

#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <AprilTags/Tag36h11.h>

namespace aprilslam {

DetectorNode::DetectorNode(const ros::NodeHandle &nh,
                           const ros::NodeHandle &pnh)
    : nh_(nh),
      tag_family_("36h11"),
      cam_calibrated_(true),
      it_(nh),
      sub_camera_(
          it_.subscribeCamera("image_raw", 1, &DetectorNode::CameraCb, this)),
      pub_detections_(nh_.advertise<sensor_msgs::Image>("detections", 1)),
      gamma_lookup_(1,256,CV_8U),
      tag_detector_(AprilTags::tagCodes36h11) {

  if (!pnh.getParam("size", tag_size_)) {
    throw std::runtime_error("No tag size specified");
  }

  const double gamma = 0.4;
  uchar* p = gamma_lookup_.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(pow(i/255.0, gamma) * 127.0);
  }
}

void DetectorNode::CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                            const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  // Show only the detection if camera is uncalibrated
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->height == 0) {
    ROS_ERROR_THROTTLE(1, "%s: %s", nh_.getNamespace().c_str(),
                       "Camera not calibrated!");
    cam_calibrated_ = false;
  }
  // Retrieve camera info and image
  model_.fromCameraInfo(cinfo_msg);

	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat image;
	cv::cvtColor(cv_ptr->image, image, CV_BGR2GRAY);

  // Detect tags
  std::vector<AprilTags::TagDetection> detections =
      tag_detector_.extractTags(image);

  cv::Mat gamma_img;
  cv::LUT(cv_ptr->image, gamma_lookup_, gamma_img);
  cv_ptr->image = gamma_img;

  // Draw detections if there are some
  if (!detections.empty()) {
    std::for_each(begin(detections), end(detections),
                  [&](const AprilTags::TagDetection &detection) {
      detection.draw(cv_ptr->image);
    });
  }
  cv::putText(cv_ptr->image, std::to_string(detections.size()), cv::Point(10, cv_ptr->image.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0,0,255), 10);
  pub_detections_.publish(cv_ptr->toImageMsg());
}

}  // namespace aprilslam

/*  @file camera_node.cpp
 *  @version
 *  @date
 *  @brief
 *  @copyright Team Surclean
*/

#include <boost/filesystem.hpp>
#include <camera_node/camera_node.h>
#include <rosbag/bag.h>

#define CAMERA_ACTION_TAKE_PICTURE 0

/* ----------------------------------------------------------------------------
 */
CameraNode::CameraNode() {}

void CameraNode::Loop() {
  // initial subscriber and publiser
  pub_image_annotation_ =
      nh_.advertise<camera_node::ImageMsg>("/SureClean_UAV/annotated_image", 1);
  trigger_server_ = nh_.advertiseService("/SureClean_UAV/take_photo",
                                         &CameraNode::TriggerImage, this);
  camera_photo_client_ =
      nh_.serviceClient<dji_sdk::CameraAction>("/dji_sdk/camera_action");
  sub_image_ = nh_.subscribe("/dji_camera/image_raw", 1,
                             &CameraNode::CameraImageCallback, this);
  // sub_gps_ =
  // nh_.subscribe("/dji_sdk/gps_position",1,&CameraNode::GPSCallback,this);
  sub_gps_ = nh_.subscribe("/rtk_gps", 1, &CameraNode::GPSCallback, this);
  sub_height_ = nh_.subscribe("/dji_sdk/height_above_takeoff", 1,
                              &CameraNode::HeightCallback, this);
  sub_attitude_ = nh_.subscribe("/dji_sdk/attitude", 1,
                                &CameraNode::AttitudeCallback, this);
  ros::spin();
}

void CameraNode::ImageProcess() {
  camera_image_msg_.header = camera_img_.header;
  camera_image_msg_.image = camera_img_;
  camera_image_msg_.gps = gps_pos_;
  camera_image_msg_.attitude = q_;
  camera_image_msg_.height = height_;
}

std::string generateImageFolder() {
  try {
    std::time_t now = std::time(0);
    std::tm *dateTimeNow = std::localtime(&now);
    int year = 1900 + dateTimeNow->tm_year;
    int month = 1 + dateTimeNow->tm_mon;
    int day = dateTimeNow->tm_mday;
    std::string imageMetaData = "imageMetaData/" + std::to_string(year) + "-" +
                                std::to_string(month) + "-" +
                                std::to_string(day) + "/";
    if (!boost::filesystem::exists("imageMetaData"))
      boost::filesystem::create_directory("imageMetaData");
    if (!boost::filesystem::exists(imageMetaData))
      boost::filesystem::create_directory(imageMetaData);
    imageMetaData += std::to_string(dateTimeNow->tm_hour) + "-" +
                     std::to_string(dateTimeNow->tm_min) + "-" +
                     std::to_string(dateTimeNow->tm_sec) + ".bag";
    return imageMetaData;
  } catch (const std::exception &e) {
    std::cout << "\nException " << e.what() << std::endl;
  }
}

bool writeRosBag(camera_node::ImageMsg &msg) {
  try {
    std::string bagFileName = generateImageFolder();
    rosbag::Bag bagWrite;
    bagWrite.open(bagFileName, rosbag::bagmode::Write);
    bagWrite.write("/SureClean_UAV/annotated_image", ros::Time::now(), msg);
    bagWrite.close();
    return true;
  } catch (const std::exception &e) {
    std::cout << "\nException " << e.what() << std::endl;
    return false;
  }
}

bool CameraNode::TriggerImage(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
  this->ImageProcess(); // update camera_image_msg_
  dji_sdk::CameraAction srvs;
  srvs.request.camera_action = CAMERA_ACTION_TAKE_PICTURE;
  if (camera_photo_client_.call(srvs)) {
    ROS_INFO("Photo has been taken!");
    if (!writeRosBag(camera_image_msg_)) {
      std::cout << "Couldn't write the following message to bag" << std::endl;
    } else {
      std::cout << "written the message to rosbag" << std::endl;
    }

  } else {
    ROS_INFO("NO Photo Taken");
  }
  pub_image_annotation_.publish(camera_image_msg_);
  ROS_INFO("Annotated Data has been Publihsed!");
  return true;
}

void CameraNode::GPSCallback(const sensor_msgs::NavSatFix &gps_msg) {
  gps_pos_ = gps_msg;
}
void CameraNode::HeightCallback(const std_msgs::Float32 &height_msg) {
  height_ = height_msg;
}
void CameraNode::AttitudeCallback(
    const geometry_msgs::QuaternionStamped &attitude_msg) {
  q_ = attitude_msg;
}
void CameraNode::CameraImageCallback(const sensor_msgs::Image &img) {
  camera_img_ = img;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_node");
  CameraNode camera_node;
  camera_node.Loop();
  return 0;
}

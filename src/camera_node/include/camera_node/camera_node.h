/*  @file camera_node.h
 *  @version 
 *  @date 
 *  @brief
 *  @copyright Team Surclean
*/

#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dji_sdk/CameraAction.h>
#include <string>
#include <geometry_msgs/QuaternionStamped.h>

#include <camera_node/ImageMsg.h>



class CameraNode 
{
public:
    CameraNode();
    /* Main Loop of execution */
    void Loop();
    /* service server to take a picture and publish annotated data */
    bool TriggerImage(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    /* API to DJI fligte control sdk to get the orientatioin and atitude */
    void ImageProcess();

private:
    ros::NodeHandle nh_;
    // Subscribers
    ros::Subscriber sub_image_;
    ros::Subscriber sub_gps_;
    ros::Subscriber sub_attitude_;
    ros::Subscriber sub_height_;
    // Publisher
    ros::Publisher pub_image_annotation_;
    // service server
    ros::ServiceServer trigger_server_;
    // service client
    ros::ServiceClient camera_photo_client_;
    // Msgs storage valie
    camera_node::ImageMsg camera_image_msg_;
    sensor_msgs::Image camera_img_;
    sensor_msgs::NavSatFix gps_pos_;
    std_msgs::Float32 height_;
    geometry_msgs::QuaternionStamped q_;
    // subscribe callback function
    void GPSCallback(const sensor_msgs::NavSatFix& gps_msg);
    void HeightCallback(const std_msgs::Float32& height_msg);
    void AttitudeCallback(const geometry_msgs::QuaternionStamped& attitude_msg);
    void CameraImageCallback(const sensor_msgs::Image& img);
};

#endif 

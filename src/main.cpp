//
// Created by axel on 22.02.2021.
//
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

//#include <sensor_msgs/PointCloud2.h>
#include "gp_ros/GPLidar.h"
#include "pcl/io/pcd_io.h"


static GpDeviceHandle g_handle = (GpDeviceHandle) GP_INVALID_DEVICE_HANDLE;
static GpBool g_bExitStreamThread = FALSE;
static pthread_t g_streamThread = (pthread_t)-1;

static GpBool g_bStreamOpen = FALSE;

static int savePic = 0;
static int picNum = 0;

pcl::PointCloud<pcl::PointXYZ> cloud;

int main(int argc, char** argv)
{
    int pc_counter = 1;
    ros::init(argc, argv, "gp_ros");

    std::string lidar_ip = "192.168.10.266";
    GPLidar gl = GPLidar(lidar_ip);
    //auto t = gl.getPointsFromFrame();
   // auto pcl =  gl.get_pcl(t);
    //pcl.makeShared(); // Hva gjør denne?
   // pcl::io::savePCDFileASCII("test_pcd.pcd", pcl);


    ros::NodeHandle nh;
    ros::NodeHandle nh1;

   // ros::param::param<std::string>("/gp/sensor_ip", lidar_ip, "IDK");
    std::string color_map;

    bool pcl_enable, depth_raw_enable, intensity_img_enable, depth_colored_image_enable, depth_gray_colored_image_enable, get_colored_images;
    ros::param::param<bool>("/gp/pointcloud_enable", pcl_enable, true);
    //  ros::param::param<bool>("/gp/depth_image_enable", depth_img_enable, 1);

    //  ros::param::param<bool>("/gp/pointcloud_enable", pcl_enable, 1);
  //  ros::param::param<bool>("/gp/intensity_image_enable", intensity_img_enable, 1);
  //  ros::param::param<bool>("/gp/depth_raw_image_enable", depth_raw_enable, 1);
  //  ros::param::param<bool>("/gp/depth_gray_colored_image_enable", depth_gray_colored_image_enable, 1);
  //  ros::param::param<bool>("/gp/depth_colored_image_enable", depth_colored_image_enable, 1);
  //  ros::param::param<std::string>("/gp/color_map", color_map, "cv::COLORMAP_RAINBOW");

  //  get_colored_images = depth_colored_image_enable || depth_gray_colored_image_enable;
    ros::Publisher pcl_pub;

    pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("gp/pcl", 10);

    while (ros::ok() && nh1.ok()) {

        if (pcl_enable) {
            auto t = gl.getPointsFromFrame();
            cloud =  gl.get_pcl(t);
        }

        if (get_colored_images) {
            //sense_lidar.get_colored_images(intensity_image, depth_image, depth_gray_image,  depth_image_colored, max_distance, distance_interval);
            //depth_gray_image.convertTo(depth_gray_image_p2, CV_8UC1);
            //cv::applyColorMap(depth_gray_image_p2, depth_image_colored, cv::COLORMAP_RAINBOW);
        } else {
            //sense_lidar.get_images(intensity_image, depth_image, max_distance);
        }

        //sense_lidar.get_intensity_pcl(sense_cloud, intensity_image);


        //intensity_image.convertTo(intensity_img_p1, CV_8UC1);
        //depth_image.convertTo(depth_img_p1, CV_8UC1);
        /*
        if (get_colored_images) {
            if (ros::param::has("/sense/depth_color_rollover")) {
                ros::param::get("/sense/depth_color_rollover", distance_interval);
            }
            depth_gray_image.convertTo(depth_gray_img_p1, CV_8UC1);
            depth_image_colored.convertTo(depth_color_img_p1, CV_8UC3);
        }


        intensity_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",intensity_img_p1).toImageMsg();
        depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_img_p1).toImageMsg();
        if (get_colored_images) {
            depth_gray_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_gray_img_p1).toImageMsg();
            depth_color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_color_img_p1).toImageMsg();
        }

        */
        cloud.header.frame_id = "gp_frame";
        cloud.header.seq = pc_counter;
        pc_counter++;
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);

        /*
        intensity_img_msg->header.frame_id = "sense_base";
        depth_img_msg->header.frame_id = "sense_base";
        if (get_colored_images) {
            depth_gray_msg->header.frame_id = "sense_base";
            depth_color_msg->header.frame_id = "sense_base";
        }
         */

        if (pcl_enable)
            pcl_pub.publish(cloud.makeShared());
        /*
        if (intensity_img_enable)
            intensity_pub.publish(intensity_img_msg);

        if (depth_raw_enable)
            depth_pub.publish(depth_img_msg);

        if (depth_gray_colored_image_enable)
            depth_color_pub.publish(depth_color_msg);

        if (depth_colored_image_enable)
            depth_gray_pub.publish(depth_gray_msg);
    */

        ros::spinOnce();
    }

    return 0;
}
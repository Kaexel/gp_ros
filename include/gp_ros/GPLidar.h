//
// Created by axel on 22.02.2021.
//
#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

extern "C" {
#include "GpSdk.h"
}


//#include <pcl_ros/point_cloud.h>
//#include <cv_bridge/cv_bridge.h>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define BOLDWHITE   "\033[1m\033[37m"
#define BOLDBLUE    "\033[1m\033[34m"


void status_check(int status, std::string error_msg);

class GPLidar {
private:
    GpDeviceHandle g_handle;
    GpCameraLensParam g_cam_param;
    int lidar_id_{};
    int integrationTime = 0;
    int integrationTime_temp = 0;
    bool bExit = false;
    GpStreamType streamType = GP_STREAM_TYPE_DEPTH;

    int status_;
    std::string lidar_ip_;
    int stream_id_{};
    long int internal_frame_count_;
    GpFrame *frames{};




public:
    explicit GPLidar(std::string &lidar_ip);
    ~GPLidar();
    std::vector<unsigned short> getPointsFromFrame();
    pcl::PointCloud<pcl::PointXYZ> get_pcl(std::vector<unsigned short> points);

};
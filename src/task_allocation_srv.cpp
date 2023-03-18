#include "ros/ros.h"

#include "task_allocation.hpp"

#include <iostream>

#include <opencv2/opencv.hpp>

int main(int argc ,char* argv[], char* env[]) {
    ros::init(argc, argv, "task_allocation_srv");
    TAServer srv;
    srv.loadMap("/home/nx/zlz_ws/src/task_allocation/MapData/map.pgm");

    cv::namedWindow("expand", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    cv::imshow("map", srv.map);
    cv::imshow("expand", srv.expanded);
    cv::waitKey(0);

    return 0;
}
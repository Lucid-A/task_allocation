#include "ros/ros.h"

#include "task_allocation.hpp"

#include <iostream>
using namespace std;

int main(int argc, char* argv[], char* env[]) {
    ros::init(argc, argv, "TAClient");
    TAClient cli;
    ros::spin();
    return 0;
}
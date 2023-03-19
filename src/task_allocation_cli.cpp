#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "task_allocation.hpp"

#include "astar.hpp"

using namespace std;

void TAClient::spin() {
    ros::spin();
}

TAClient::TAClient() : loop_rate(10)
{
    this->pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/my_goal", 1000);
    this->sub_status = nh.subscribe("status", 1000, &TAClient::status_callback, this);
    this->sub_pose = nh.subscribe("/pose/robot_pose_ekf", 1000, &TAClient::pose_callback, this);
    this->sub_task = nh.subscribe("/task_result/ocr", 1000, &TAClient::task_callback, this);

    this->cli_statusChange = nh.serviceClient<task_allocation::statusChange>("statusChange");
    this->cli_taskFinished = nh.serviceClient<task_allocation::taskFinished>("taskFinished");

    boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtr;
    sharedPtr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose/robot_pose_ekf", nh);

    this->car.id = 1;
    this->car.sensor_status = {true, true};
    this->car.pose = sharedPtr->pose;

    ROS_INFO("Car Inited");
}

TAClient::~TAClient() {
    // pass
}

void TAClient::task_callback(const std_msgs::String::ConstPtr &msg)
{
    string result(msg->data);
    
    this->srv_taskFinished.request.car = this->car;
    this->srv_taskFinished.request.finished_task = this->task_list.at(0);
    this->srv_taskFinished.request.result.data = result;

    ROS_INFO("Task finished, result: %s", result.c_str());

    if (cli_taskFinished.call(this->srv_taskFinished))
    {
        this->task_list.clear();
        this->task_list = this->srv_taskFinished.response.task_list;
        if (this->task_list.size() > 0)
        {
            geometry_msgs::PoseStamped ps;
            ps.header.seq = 0;
            ps.header.stamp = ros::Time::now();
            ps.header.frame_id = "tag";
            ps.pose = this->task_list.at(0).pose;
            this->pub_goal.publish(ps);
        }
    } else {
        ROS_INFO("Failed to connect to Task Finished service!");
    }
}

void TAClient::status_callback(const std_msgs::Int64::ConstPtr &msg)
{
    int sensor_index = msg->data >> 32;
    int sensor_status = (int)(msg->data && 0x00000000ffffffff);
    if (this->car.sensor_status.at(sensor_index) == sensor_status)
        return;

    ROS_INFO("Update sensor %d status: from %d to %d", sensor_index, this->car.sensor_status.at(sensor_index), sensor_status);
    this->car.sensor_status.at(sensor_index) = sensor_status;
    this->srv_statusChange.request.car = this->car;

    if (this->cli_statusChange.call(this->srv_statusChange)) {
        this->task_list.clear();
        this->task_list = this->srv_statusChange.response.task_list;
        if (this->task_list.size() > 0)
        {
            geometry_msgs::PoseStamped ps;
            ps.header.seq = 0;
            ps.header.stamp = ros::Time::now();
            ps.header.frame_id = "tag";
            ps.pose = this->task_list.at(0).pose;
            this->pub_goal.publish(ps);
        }
    }
    else
    {
        ROS_INFO("Failed to connect to Task Finished service!");
    }
}

void TAClient::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    this->car.pose = msg->pose;
//    ROS_INFO("Get robot pose: (%.6f, %.6f) dir: %.0f", msg->pose.position.x, msg->pose.position.y, acos(msg->pose.orientation.w) / 3.14 * 180);
}

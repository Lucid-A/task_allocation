#pragma once

#include <map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
//#include "move_base_msgs/MoveBaseActionGoal.h"

#include "task_allocation/Car.h"
#include "task_allocation/Task.h"
#include "task_allocation/registerCar.h"
#include "task_allocation/statusChange.h"
#include "task_allocation/taskFinished.h"

#include "SA.hpp"

class TAServer
{
public:
    ros::NodeHandle nh;
    ros::Rate loop_rate;
    ros::ServiceServer srv_registerCar;
    ros::ServiceServer srv_statusChange;
    ros::ServiceServer srv_taskFinished;

    cv::Mat map;
    cv::Mat expanded;

    std::map<int, int> car_id2index;
    std::map<int, int> task_id2index;
    
    // use index
    std::vector<task_allocation::Car> car_list;
    std::vector<task_allocation::Task> task_list;

    // index -> id
    std::vector<int> tasks_to_be_done;

    // use id
    std::map<int, std::string> tasks_done; // task_id -> result
    std::map<int, int> tasks_assigned; // task_id -> car_id
    

    SA sa;

public:
    TAServer();
    ~TAServer();

    virtual float CalculateCostBetween2Points(float start_x, float start_y, float end_x, float end_y);

    bool loadMap(std::string map_file);
    int loadTask(float x, float y, float dir_x, float dir_y, int type);

    cv::Mat calculateCostMap();

    bool registerCar(
        task_allocation::registerCar::Request & req,
        task_allocation::registerCar::Response & res
    );
    bool statusChange(
        task_allocation::statusChange::Request & req,
        task_allocation::statusChange::Response & res
    );
    bool taskFinished(
        task_allocation::taskFinished::Request &req,
        task_allocation::taskFinished::Response &res
    );
};

class TAClient
{
public:
    ros::NodeHandle nh;
    ros::Rate loop_rate;

    ros::Publisher pub_goal;

    ros::Subscriber sub_pose;
    ros::Subscriber sub_task;
    ros::Subscriber sub_status;

    ros::ServiceClient cli_statusChange;
    ros::ServiceClient cli_taskFinished;

    task_allocation::statusChange srv_statusChange;
    task_allocation::taskFinished srv_taskFinished;

    task_allocation::Car car;
    std::vector<task_allocation::Task> task_list;

public:
    TAClient();
    ~TAClient();

    void status_callback(const std_msgs::Int64::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void task_callback(const std_msgs::String::ConstPtr &msg);
};
#include <iostream>

#include "ros/ros.h"

#include "task_allocation.hpp"

#include "astar.hpp"

using namespace std;

float TAServer::CalculateCostBetween2Points(float start_x, float start_y, float end_x, float end_y)
{
    return CalculateA(this->expanded, start_x, start_y, end_x, end_y);
}

bool TAServer::loadMap(string map_file) {
    this->map = cv::imread(map_file, cv::IMREAD_GRAYSCALE);

    if (!this->map.data)
    {
        cout << "No image data" << endl;
        return false;
    }

    double thresh = 230;
    double maxval = 255;
    cv::threshold(this->map, this->map, thresh, maxval, cv::THRESH_BINARY);
    cout << "threshold" << endl;

    // Make Obstacle White(255) and Road Black(0)
    cv::bitwise_not(this->map, this->map);
    cout << "not" << endl;

    // Do obstacle expansion
    int dilation_size = 6;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * dilation_size, 2 * dilation_size),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(this->map, this->expanded, element, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    cout << "dilate" << endl;
    return true;
}

cv::Mat TAServer::calculateCostMap()
{
    int taskNum = this->tasks_to_be_done.size();
    int carNum = this->car_list.size();

    cv::Mat cost_map(taskNum + carNum, taskNum, CV_32FC1);

    for (int i = 0; i < taskNum; ++i)
    {
        task_allocation::Task& task_start = task_list.at(i);
        for (int j = 0; j < i; ++j)
        {
            task_allocation::Task& task_end = task_list.at(j);

            cost_map.at<float>(i, j) = this->CalculateCostBetween2Points(task_start.pose.position.x, task_start.pose.position.y, task_end.pose.position.x, task_end.pose.position.y);
            cost_map.at<float>(j, i) = cost_map.at<float>(i, j);
        }
    }

    for (int i = taskNum; i < taskNum + carNum; ++i)
    {
        task_allocation::Car &car = car_list.at(i);
        for (int j = 0; j < taskNum; ++j)
        {
            task_allocation::Task& task = task_list.at(j);
            int task_type = task.type - 'A';
            if (car.sensor_status.at(task_type))
            {
                cost_map.at<float>(i, j) = this->CalculateCostBetween2Points(car.pose.position.x, car.pose.position.y, task.pose.position.x, task.pose.position.y);
            } else {
                cost_map.at<float>(i, j) = -1.0f;
            }
        }
    }
    return cost_map;
}

TAServer::TAServer() : loop_rate(10) {
    this->srv_registerCar = nh.advertiseService("registerCar", &TAServer::registerCar, this);
    this->srv_statusChange = nh.advertiseService("statusChaneg", &TAServer::statusChange, this);
    this->srv_taskFinished = nh.advertiseService("taskFinished", &TAServer::taskFinished, this);
    ROS_INFO("Services statred!");
    ros::spin();
}

TAServer::~TAServer()
{
     // pass
}

bool TAServer::registerCar(
    task_allocation::registerCar::Request &req,
    task_allocation::registerCar::Response &res) 
    {
        for (auto& c : this->car_list) {
            if (c.id == req.car.id) {
                ROS_INFO("Car %d already exists!", req.car.id);
                ROS_INFO("Update its status:");
                ROS_INFO("Old Status:");
                cout << c << endl;
                ROS_INFO("New Status:");
                cout << req.car << endl;
                c = req.car;
                res.status.data = true;
                return true;
            }
        }

        this->car_list.push_back(req.car);
        res.status.data = true;
        return true;
    }

bool TAServer::statusChange(
    task_allocation::statusChange::Request &req,
    task_allocation::statusChange::Response &res)
    {
        int car_index;
        task_allocation::Car &car = req.car;
        //map<int, int>::const_iterator ret = this->car_id2index.find(car.id);
        auto ret = this->car_id2index.find(car.id);

        if (ret != this->car_id2index.end())
        {
            car_index = car_list.size() - 1;
            car_list.push_back(car);
            this->car_id2index[car.id] = car_list.size();

            ROS_INFO("Register new car[%d] id: %d", car_index, car.id);
            ROS_INFO("Status:");
            cout << car << endl;
        }
        else
        {
            car_index = (*ret).second;
            task_allocation::Car &c = this->car_list.at(car_index);
            ROS_INFO("Update Car[%d] id: %d status:", car_index, car.id);
            ROS_INFO("Old Status:");
            cout << c << endl;
            ROS_INFO("New Status:");
            cout << car << endl;
            this->car_list.at(car_index) = car;
        }

        this->sa.InitSA(this->calculateCostMap());
        this->sa.StartSA();

        for (auto &t : this->sa.splited_paths.at(car_index))
        {
            res.task_list.push_back(task_list.at(t));
        }

        int task_index = this->sa.splited_paths.at(car_index).at(0);
        tasks_assigned.insert(make_pair(task_list.at(task_index).id, car.id));
        tasks_to_be_done.erase(tasks_to_be_done.begin() + task_index);

        return true;
    }

bool TAServer::taskFinished(
    task_allocation::taskFinished::Request &req,
    task_allocation::taskFinished::Response &res)
    {
        int car_index;
        task_allocation::Car &car = req.car;
        // map<const int, int>::const_iterator ret = this->car_id2index.find(car.id);
        auto ret = this->car_id2index.find(car.id);

        task_allocation::Task &task = req.finished_task;

        if (ret != this->car_id2index.end())
        {
            ROS_INFO("REJECTED: Unknown car id: %d finshed task%d %d", car.id, task.type, task.id);
            return false;
        }

        car_index = (*ret).second;
        task_allocation::Car &c = this->car_list.at(car_index);
        ROS_INFO("Update Car[%d] id: %d status:", car_index, car.id);
        ROS_INFO("Old Status:");
        cout << c << endl;
        ROS_INFO("New Status:");
        cout << car << endl;
        this->car_list.at(car_index) = car;

        tasks_assigned.erase(task.id);
        string result = req.result.data;
        if (result.empty())
        {
            tasks_to_be_done.push_back(task.id);
        }
        else
        {
            tasks_done.insert(make_pair(task.id, string(req.result.data)));
        }

        this->sa.InitSA(this->calculateCostMap());
        this->sa.StartSA();

        for (auto &t : this->sa.splited_paths.at(car_index))
        {
            res.task_list.push_back(task_list.at(t));
        }

        int task_index = this->sa.splited_paths.at(car_index).at(0);
        tasks_assigned.insert(make_pair(task_list.at(task_index).id, car.id));
        tasks_to_be_done.erase(tasks_to_be_done.begin() + task_index);

        return true;
    }


TAClient::TAClient() : loop_rate(10)
{
    this->pub_goal = nh.advertise<geometry_msgs::PoseStamped>("my_goal", 1000);
    this->sub_status = nh.subscribe("status", 1000, &TAClient::status_callback, this);
    this->sub_pose = nh.subscribe("robot_pose", 1000, &TAClient::pose_callback, this);
    this->sub_task = nh.subscribe("task_result", 1000, &TAClient::task_callback, this);

    this->cli_statusChange = nh.serviceClient<task_allocation::statusChange>("Status change");
    cli_taskFinished = nh.serviceClient<task_allocation::taskFinished>("Task finished");

    this->car.id = 1;
    this->car.sensor_status = {true, true};

    ros::spin();
}

TAClient::~TAClient() {
    // pass
}

void TAClient::task_callback(const std_msgs::String::ConstPtr &msg)
{
    string result(msg->data);
    
    srv_taskFinished.request.car = this->car;
    srv_taskFinished.request.finished_task = this->task_list.at(0);
    srv_taskFinished.request.result.data = result;

    if (cli_taskFinished.call(srv_taskFinished))
    {
        this->task_list.clear();
        this->task_list = srv_taskFinished.response.task_list;
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

    ROS_INFO("Update sensor status: from %d to %d", this->car.sensor_status.at(sensor_index), sensor_status);
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
}
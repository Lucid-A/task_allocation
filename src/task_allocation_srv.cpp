#include <cmath>
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

    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    cv::imshow("map", this->map);
    cv::waitKey(10000);

    // Make Obstacle White(255) and Road Black(0)
    cv::bitwise_not(this->map, this->map);
    cout << "not" << endl;

    cout << "in func: " << &this->map << endl;
    
    cv::imwrite("/home/nx/Desktop/1.png", this->map);
    cv::imshow("map", this->map);

    // Do obstacle expansion
    int dilation_size = 6;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * dilation_size, 2 * dilation_size),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(this->map, this->expanded, element, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    cout << "dilate" << endl;

    cv::namedWindow("expand", cv::WINDOW_AUTOSIZE);
    cv::imwrite("/home/nx/Desktop/2.png", this->expanded);
    cv::imshow("expand", this->expanded);

    cv::waitKey(0);

    return true;
}

int TAServer::loadTask(float x, float y, float dir_x, float dir_y, int type) {
    int task_id = this->task_list.size();

    task_allocation::Task task;
    task.id = task_id,
    task.type = type,
    task.pose.position.x = x,
    task.pose.position.y = y,
    task.pose.position.z = 0,
    task.pose.orientation.w = cos(atan2(dir_y, dir_x) / 2),
    task.pose.orientation.x = 0,
    task.pose.orientation.y = 0,
    task.pose.orientation.z = sin(atan2(dir_y, dir_x) / 2);

    this->task_list.push_back(task);

    return task_id;
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
    this->srv_statusChange = nh.advertiseService("statusChange", &TAServer::statusChange, this);
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

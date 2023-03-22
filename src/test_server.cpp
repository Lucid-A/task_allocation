#include "ros/ros.h"

#include "task_allocation.hpp"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
using namespace std;

void pic2map(float x, float y, float& X, float& Y) {
    X = 3 * (x + 10);
    Y = 3 * (y + 10);
}

void map2tag(float x, float y, float &X, float &Y) {
    float tag_origin_in_map_x = 30.6;
    float tag_origin_in_map_y = 38.6;
    X = (-x + tag_origin_in_map_x) * 5;
    Y = (-y + tag_origin_in_map_y) * 5;
}

void map2pic(float x, float y, float& X, float& Y) {
    X = x / 3 - 10;
    Y = y / 3 - 10;
}

void tag2pic(float x, float y, float& X, float& Y) {
    float tag_origin_in_map_x = 30.6;
    float tag_origin_in_map_y = 38.6;
    X = -x / 5 + tag_origin_in_map_x;
    Y = -y / 5 + tag_origin_in_map_y;
}

int main(int argc, char *argv[], char *env[])
{
    ros::init(argc, argv, "task_allocation_srv");

    //float tasks[][4] = {
    //    { 4,24,  1,  0},
    //    {11,24, -1,  0},
    //    {16,39,  1,  0},
    //    {23,39, -1,  0},
    //    {33,16,  0, -1},
    //    {33, 8,  0,  1}
    //};
    // float tasks[][4] = {
    //     { -0.2375534306976562,  4.914732823534101,  0, -1},
    //     { -0.5962731815435898,  3.349024511404746,  0,  1},
    //     { 2.4670310132855744,  1.9470714065020651,  1,  0},
    //     { 3.8851847486218674,  1.9398198615820275, -1,  0},
    //     { 2.5090954373381193, -0.1600137717979262, -1,  0},
    //     { 0.9352839396192938, 0.10694789725441556,  1,  0}
    // };

    // float tasks[][4] = {
    //     {-0.237,  4.914,  0, -1},
    //     {-0.596,  3.349,  0,  1},
    //     { 2.467,  1.947,  1,  0},
    //     { 3.885,  1.939, -1,  0},
    //     { 2.760, -0.150, -1,  0},
    //     { 0.850, -0.150,  1,  0}
    // };

    vector<vector<float>> tasks;
    ifstream ifs("/home/nx/zlz_ws/src/task_allocation/src/tasks.csv", ios::in);
    string line;

    while(getline(ifs, line)) {
        cout << "Parsing: " << line << endl;
        stringstream ss(line);
        string str;

        vector<float> task;
        char ch;
        float num;

        ss >> num;
        task.push_back(num);
        for (int i = 0; i < 3; ++i)
        {
            ss >> ch >> num;
            task.push_back(num);
        }

        for (int i = 0; i < task.size(); ++i) {
            cout << task.at(i) << " ";
        }
        cout << endl;

        tasks.push_back(task);
    }

    TAServer srv;
    srv.loadMap("/home/nx/zlz_ws/src/task_allocation/MapData/map.pgm");
    for (int i = 0; i < 6; ++i) {
        int ret = srv.loadTask(tasks[i][0], tasks[i][1], tasks[i][2], tasks[i][3], 'A');
        cout << i << ' ' << ret << endl;
    }

    cout << "out of func: " << &srv.map << endl;
    ros::spin();

    return 0;
}

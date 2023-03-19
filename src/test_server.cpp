#include "ros/ros.h"

#include "task_allocation.hpp"

#include <iostream>

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

    float tasks[][4] = {
        { 4,24,  1,  0},
        {11,24, -1,  0},
        {16,39,  1,  0},
        {23,39, -1,  0},
        {33,16,  0, -1},
        {33, 8,  0,  1}
    };

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
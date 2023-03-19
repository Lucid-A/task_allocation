#include <string>
#include <iostream>
#include "ros/ros.h"
#include "ros/master.h"

int main(int argc, char* argv[], char* env[]) {
    for (int i = 0; i < argc; ++i) {
        std::cout << "arg[" << i << "] = " << argv[i] << std::endl;
    }

    std::cout << std::endl;

    int i = 0;
    char *str = env[0];
    while (str) {
        std::cout << "env[" << i << "] = " << str << std::endl;
        ++i;
        str = env[i];
    }

    ros::init(argc, argv, "get_topics");
    ros::NodeHandle nh;

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (const auto& [name, datatype] : master_topics) {
        if (name.substr(name.size()-3, name.size()) == std::string("agg"))
            std::cout << "Topic: " << name << " DataType: " << datatype << std::endl;
    }
    return 0;
}
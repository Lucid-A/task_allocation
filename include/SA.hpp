#pragma once

#include <cmath>

#include <vector>
#include <algorithm>

#include <random>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "task_allocation/Car.h"
#include "task_allocation/Task.h"

class SA
{
private:
    std::random_device seed;
    std::mt19937 gen;

public:
    // 节点之间的代价数据
    cv::Mat costs;
    int carNum;
    int taskNum;

public:
    // 模拟退火算法参数
    int max_iter;   // 最大迭代次数
    int Lk;         // 每个温度下的迭代次数
    double T0;      // 初始温度
    double Tc;      // 当前温度
    double alpha;   // 温度衰减系数
    
    // 过程数据记录
    float best_cost;
    std::vector<int> best_path;  // 最佳路径
    std::vector<std::vector<int>> splited_paths; // 分割好的路径结果

    SA();
    SA(const cv::Mat &cost_map);

    ~SA();
    
    // 返回一个概率值 p ∈ [0,1]
    float P();

    void InitSA(const cv::Mat &cost_map);
    void StartSA();
    // 初始化运输路径
    float InitPath(std::vector<int> &init_path);
    // 模拟退火算法生成新解
    float NewPath(std::vector<int> old_path, std::vector<int> &new_path);
    // 分割路径
    void SplitPath(std::vector<int> &path);
    // 计算分割好的路径的总代价
    float CalculateCost(std::vector<int> &path);

    void Fisher_yates_Shuffle(std::vector<int> &arr, std::vector<int> &res);
    void Knuth_Durstenfeld_Shuffle(std::vector<int> &arr);
};
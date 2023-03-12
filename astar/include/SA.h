#ifndef _SA_H_
#define _SA_H_
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <time.h>
#include "stdlib.h"
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

#define NodeCount 21    // 结点个数
#define AllNodeCount 69 // 包含路的总结点个数
#define MaxTaskNum 10   // 最大任务数量
#define MaxCarNum 5     // 最大车数量

// 分配车辆策略
typedef enum
{
    MINDISTANCE = 0, // 最小总里程
    MINTIME = 1,     // 完成任务最短时间
} Strategy;

// 运输任务结构体
struct Task
{
    int TaskNum = -1;    // 任务编号
    float X = 0, Y = 0;  // 任务位置
    char TaskType = 'A'; // 任务类型1-A,2-B
};

// 车辆结构体
struct Car
{
    int CarNum = -1;        // 车辆编号
    float X = 0, Y = 0;     // 车辆位置
    int Status[2] = {1, 1}; // 车辆状态
    std::vector<Task> path; // 车辆路径
    float cost = 0.0;
};

extern int Location[AllNodeCount][2];        // 外部声明坐标
extern int Path[AllNodeCount][AllNodeCount]; // 外部声明路径信息

class SA
{
public:
    Strategy strategy;

    // 模拟退火算法参数：初始温度、当前温度、最大迭代次数、每个温度下的迭代次数、温度衰减系数
    double T0, T, alpha;
    int max_iter, Lk;

    std::vector<int> init_path;  // 初始路径
    std::vector<int> best_path;  // 初始路径
    std::vector<int> task_index; // 任务点编号
    std::vector<float> best_cost_epoch;
    float best_cost;

    SA(Strategy s);
    ~SA();
    void StartSA(cv::Mat &map, int CarNum, Car *car, int TaskNum, Task *task);                  // 开始进行模拟退火算法
    void InitPath(int CarNum, int TaskNum, Task *task);                                         // 初始化运输路径
    void SplitPath(int CarNum, Car *car, Task *task, std::vector<int> &path);                   // 分割路径
    bool VerifySolution(std::vector<int> &path, int CarNum, Car *car, int TaskNum, Task *task); // 检查解是否满足要求
    float CalculateCost(cv::Mat &map, int CarNum, Car *car);                                    // 代价计算，通过调配策略决定
    void NewPath(std::vector<int> oldpath, std::vector<int> &newpath);                          // 模拟退火算法生成新解
};
float CalculateD(float x0, float y0, float x1, float y1); // 计算两点距离
// void Floyd();//floyd算法求最短路
void RandomArray(std::vector<int> oldArray, std::vector<int> &newArray); // 随机打乱数组

// #ifndef _NAVIGATION_H_
// #define _NAVIGATION_H_
//
// #include <iostream>
// #include <fstream>
// #include <iomanip>
// #include <math.h>
// #include <time.h>
//
// #define B_NodeNum			132		//0-131为本部结点
// #define NodeCount			181		//132-180为沙河结点
// #define B_SchoolBusNodeNum	114		//本部校车结点编号
// #define B_BusNodeNum		131		//本部公交汽车结点编号
// #define S_SchoolBusNodeNum	165		//沙河校车结点编号
// #define S_BusNodeNum		180		//沙河公交汽车结点编号
// #define MaxValue			99999999//初始最短距离或最短时间
// #define IteCount			1000	//迭代次数
// #define AntCount			100		//蚂蚁数量，循环搜索
//
// struct Node
//{
//	std::string Name;//结点名字
//	int LX, LY, PX, PY;//逻辑位置和实际位置
//	int ConnectionNodeNum[4];//周围的结点编号
//	double ConnectionNodeDistance[4];//到周围结点的距离
//	double Crowed;//该结点的拥挤程度
//	bool RideAllow;//该结点是否允许骑行自行车
//	bool ServiceNode;
// };
//
// struct Road
//{
//	int NodeNum;//结点编号
//	int LX, LY;//结点位置
//	int MoveDirection;//向下一个结点的移动方向8246对应上下左右
//	double Crowed;//该节点的拥挤程度
//	int WaitCount;//等待时长计数器。用来表示模拟行进的人物在对应位置的等待时间
// };
//
// class Ant
//{
// public:
//	Node MapNodes[NodeCount];//所有的节点信息
//	bool CityVisited[NodeCount];//所有结点的访问信息
//	int TravelCityNum[NodeCount * 2];//走过的结点信息
//	int TravelCityCount;//走过的所有的结点数量----存储最佳路径时可以用来作为中断标志，进行多个校区之间的导航连接
//	int CurrentCity;//当前所在结点编号
//	double MoveSpeed;//移动速度--用于计算时间------最终速度为默认移动速度除以结点方向上的拥挤程度
//	double MoveDistance;//移动的距离
//	double MoveTime;//移动花费的时间
//	Ant();
//	~Ant();
//	void ResetAnt();//重置蚂蚁信息---除了地图结点信息，全部重置
//	int FindMinTime(int, int);
//	int SelectNextCityNum(bool);//选择下一个前进的结点----先寻找到一条路径到达终点，然后再计算距离和时间的优先程度
//	void MoveAnt(bool, int);//移动蚂蚁位置，并计算移动距离和花费时间
//	void Search(int, int, bool, int);//搜索一条起点到终点的路径出来。根据布尔变量判断是否更改移动速度
// };
//
// class Navigation
//{
// public:
//	Ant SearchAnt;//用来搜索路径的蚂蚁
//	Ant BestAnt;//用来存储最佳路径的蚂蚁
//	Navigation();
//	~Navigation();
//	bool returnCross(int, int);
//	void ResetResult();//如果有更短的路径出现。则更新一定区域内的结点信息
//	void Search1(int, int, bool, int);//策略一：最短路径。搜索同一校区内任意不同两点之间的最短路径路线出来
//	void Search2(int, int, bool, int);//策略二：最短时间。搜索同一校区内任意不同两点之间的最短时间路线出来
//	void SearchTime(int, int, bool, int);
// };
//
#endif

#include "SA.h"
#include "astar.hpp"

float CalculateD(float x0, float y0, float x1, float y1)
{
    return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}
void RandomArray(std::vector<int> oldArray, std::vector<int> &newArray)
{
    // 随机打乱
    for (int i = oldArray.size(); i > 0; i--)
    {
        srand(unsigned(time(NULL)));
        // 选中的随机下标
        int index = rand() % i;
        // 根据选中的下标将原数组选中的元素push到新数组
        newArray.push_back(oldArray[index]);
        // 将原数组中选中的元素剔除
        oldArray.erase(oldArray.begin() + index);
    }
}
SA::SA(Strategy s)
{
    // 算法参数
    T0 = 1000;
    T = T0;
    max_iter = 200;
    Lk = 20;
    alpha = 0.95;
    strategy = s;
}
SA::~SA()
{
}

bool SA::VerifySolution(std::vector<int> &path, int CarNum, Car *car, int TaskNum, Task *task)
{
    SplitPath(CarNum, car, task, path);
    for (int i = 0; i < CarNum; i++)
    {
        if (car[i].Status[0] == 0)
        {
            for (int j = 0; j < car[i].path.size(); j++)
            {
                if (car[i].path[j].TaskType == 'A')
                {
                    return false;
                }
            }
        }
        else if (car[i].Status[1] == 0)
        {
            for (int j = 0; j < car[i].path.size(); j++)
            {
                if (car[i].path[j].TaskType == 'B')
                {
                    return false;
                }
            }
        }
    }
    return true;
}
void SA::InitPath(int CarNum, int TaskNum, Task *task) // 初始化运输路径
{
    for (int i = 0; i < TaskNum; i++)
    {
        task_index.push_back(task[i].TaskNum);
    }
    RandomArray(task_index, init_path);
    for (int i = 0; i < CarNum - 1; i++)
    {
        init_path.insert(init_path.begin() + (rand() % (init_path.size() + 1)), -1); // 用-1标记分隔路径
    }
}
void SA::SplitPath(int CarNum, Car *car, Task *task, std::vector<int> &path) // 分割路径
{
    std::vector<int> split_index;
    for (int i = 0; i < path.size(); i++)
    {
        if (path[i] == -1)
        {
            split_index.push_back(i);
        }
    }
    split_index.insert(split_index.begin(), 0);
    split_index.insert(split_index.end(), path.size());
    for (int i = 0; i < CarNum; i++)
    {
        car[i].path.clear();
        for (int j = split_index[i]; j < split_index[i + 1]; j++)
        {
            if (path[j] != -1)
            {
                int index = std::find(task_index.begin(), task_index.end(), path[j]) - task_index.begin();
                car[i].path.push_back(task[int(index)]);
            }
        }
    }
}
float SA::CalculateCost(cv::Mat &map, int CarNum, Car *car)
{
    std::vector<float> each_cost;
    for (int i = 0; i < CarNum; i++)
    {
        float cost = 0;
        if (car[i].path.size() == 0) // 不执行任何任务点，cost为0
        {
        }
        else // 计算执行任务点的代价
        {
            // cost += CalculateD(car[i].X, car[i].Y, car[i].path[0].X, car[i].path[0].Y);
            cost += CalculateA(map, car[i].X, car[i].Y, car[i].path[0].X, car[i].path[0].Y);
            for (int j = 0; j < car[i].path.size() - 1; j++)
            {
                // cost += CalculateD(car[i].path[j].X, car[i].path[j].Y, car[i].path[j + 1].X, car[i].path[j + 1].Y);
                cost += CalculateA(map, car[i].path[j].X, car[i].path[j].Y, car[i].path[j + 1].X, car[i].path[j + 1].Y);
            }
        }
        each_cost.push_back(cost);
        car[i].cost = cost;
    }
    // 计算最小距离代价
    if (strategy == MINDISTANCE)
    {
        float costs = 0;
        for (int i = 0; i < CarNum; i++)
            costs += each_cost[i];
        return costs;
    }
    // 计算最短时间代价
    else // if (strategy == MINTIME)
    {
        float costs = 0;
        for (int i = 0; i < CarNum; i++)
            costs += each_cost[i];
        return *std::max_element(each_cost.begin(), each_cost.end()) + 0.1 * costs;
    }
}
void SA::NewPath(std::vector<int> oldpath, std::vector<int> &newpath) // 模拟退火算法生成新解
{
    // 三种生成新解的方式
    newpath.clear();
    float p = rand() / (RAND_MAX + 1.0);
    // 0.25的概率交换法
    // 0.25的概率移位法
    // 0.25的概率倒置法
    if (p < 0.25)
    {
        int id1 = rand() % oldpath.size();
        int id2 = rand() % oldpath.size();
        while (id1 == id2)
        {
            id1 = rand() % oldpath.size();
            id2 = rand() % oldpath.size();
        }
        std::swap(oldpath[id1], oldpath[id2]);
        newpath = oldpath;
    }
    else if (p < 0.5)
    {
        int id1 = rand() % oldpath.size();
        int id2 = rand() % oldpath.size();
        int id3 = rand() % oldpath.size();
        while (id1 == id2 || id1 == id3 || id2 == id3)
        {
            id1 = rand() % oldpath.size();
            id2 = rand() % oldpath.size();
            id3 = rand() % oldpath.size();
        }
        std::vector<int> idx = {id1, id2, id3};
        std::sort(idx.begin(), idx.end());
        newpath.insert(newpath.end(), oldpath.begin(), oldpath.begin() + idx[0]);
        newpath.insert(newpath.end(), oldpath.begin() + idx[1], oldpath.begin() + idx[2]);
        newpath.insert(newpath.end(), oldpath.begin() + idx[0], oldpath.begin() + idx[1]);
        newpath.insert(newpath.end(), oldpath.begin() + idx[2], oldpath.end());
    }
    else if (p < 0.75)
    {
        int id1 = rand() % oldpath.size();
        int id2 = rand() % oldpath.size();
        int id3 = rand() % oldpath.size();
        while (id1 == id2 || id1 == id3 || id2 == id3)
        {
            id1 = rand() % oldpath.size();
            id2 = rand() % oldpath.size();
            id3 = rand() % oldpath.size();
        }
        std::vector<int> idx = {id1, id2, id3};
        std::sort(idx.begin(), idx.end());
        newpath.insert(newpath.end(), oldpath.begin(), oldpath.begin() + idx[0]);
        newpath.insert(newpath.end(), oldpath.begin() + idx[1] + 1, oldpath.begin() + idx[2] + 1);
        newpath.insert(newpath.end(), oldpath.begin() + idx[0], oldpath.begin() + idx[1] + 1);
        newpath.insert(newpath.end(), oldpath.begin() + idx[2] + 1, oldpath.end());
    }
    else
    {
        int id1 = rand() % (oldpath.size() + 1);
        int id2 = rand() % (oldpath.size() + 1);
        while (id1 >= id2)
        {
            id1 = rand() % oldpath.size();
            id2 = rand() % oldpath.size();
        }
        std::reverse(oldpath.begin() + id1, oldpath.begin() + id2);
        newpath = oldpath;
    }
}
void SA::StartSA(cv::Mat &map, int CarNum, Car *car, int TaskNum, Task *task) // 开始模拟退火
{
    int break_flag = 0;
    int rectify_flag = 0; // 防止丢失最优解标志
    InitPath(CarNum, TaskNum, task);
    while (VerifySolution(init_path, CarNum, car, TaskNum, task) == false)
    {
        break_flag++;
        if (break_flag > 100)
        {
            break_flag = 0;
            std::cout << "No suitable path in this condition!" << std::endl;
            break;
        }
        InitPath(CarNum, TaskNum, task);
    }
    break_flag = 0; // 防止死循环标志
    float init_cost = CalculateCost(map, CarNum, car);
    float current_cost = init_cost;
    best_cost = init_cost;

    std::vector<int> current_path = init_path; // 当前路径
    std::vector<int> new_path;                 // 新路径

    for (int iter = 0; iter < max_iter; iter++) // 温度迭代
    {
        for (int k = 0; k < Lk; k++) // 每个温度下的迭代
        {
            NewPath(current_path, new_path);
            while (VerifySolution(new_path, CarNum, car, TaskNum, task) == false)
            {
                std::cout << k << std::endl;
                break_flag++;
                if (break_flag > 100)
                {
                    break_flag = 0;
                    std::cout << "No suitable path in this condition!" << std::endl;
                    break;
                }
                NewPath(current_path, new_path);
            }
            break_flag = 0;
            float new_cost = CalculateCost(map, CarNum, car);
            float p_update = rand() / (RAND_MAX + 1.0);
            if (new_cost < current_cost || p_update < exp(-(new_cost - current_cost) / T)) // 根据Metropolis准则计算一个概率更新解
            {
                current_path = new_path;
                current_cost = new_cost;
            }
            if (current_cost < best_cost)
            {
                rectify_flag = 0;
                best_path = current_path;
                best_cost = current_cost;
            }
        }
        rectify_flag++;
        best_cost_epoch.push_back(best_cost);
        if (rectify_flag == 5)
        {
            rectify_flag = 0;
            current_path = best_path;
            current_cost = best_cost;
        }
        T = T * alpha;
    }
    SplitPath(CarNum, car, task, best_path);
}

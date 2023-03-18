#include "SA.hpp"
using namespace std;

void SA::Fisher_yates_Shuffle(std::vector<int>& arr, std::vector<int>& res) {
    uniform_int_distribution<> dis(0, arr.size() - 1);

    for (int i = 0; i < arr.size(); ++i) {
        int rand_index = dis(this->gen);
        res.push_back(arr[rand_index]);
        arr.erase(arr.begin() + rand_index);
    }
}

void SA::Knuth_Durstenfeld_Shuffle(std::vector<int>& arr) {
    uniform_int_distribution<> dis(0, arr.size() - 1);

    for (int i = arr.size() - 1; i >= 0; --i) {
        uniform_int_distribution<> dis(0, i + 1);
        std::swap(arr[dis(this->gen)], arr[i]);
    }
}

float SA::P() {
    uniform_real_distribution<> dis_p(0.0f, 1.0f);
    return dis_p(this->gen);
}

SA::SA() : gen(seed())
{
    // 算法参数
    T0 = 1000;
    Tc = T0;
    max_iter = 200;
    Lk = 20;
    alpha = 0.95;
}

SA::SA(const cv::Mat &cost_map) : gen(seed())
{
    // 算法参数
    T0 = 1000;
    Tc = T0;
    max_iter = 200;
    Lk = 20;
    alpha = 0.95;
    this->InitSA(cost_map);
}

SA::~SA() {
    // pass
}

void SA::InitSA(const cv::Mat &cost_map) {
    auto s = cost_map.size();
    this->taskNum = s.width;
    this->carNum = s.height - this->taskNum;

    this->costs = cost_map.clone();
}

// 开始模拟退火
void SA::StartSA() 
{
    int rectify_flag = 0; // 防止丢失最优解标志
    vector<int> init_path;

    // 找到一条满足约束条件的初始路径
    float init_cost = this->InitPath(init_path);
    
    if (init_cost < 0.0f) {
        cout << "No suitable path in this condition!" << endl;
        return;
    }

    best_cost = init_cost;
    float current_cost = init_cost;

    std::vector<int> current_path = init_path; // 当前路径
    std::vector<int> new_path;                 // 新路径

    for (int iter = 0; iter < max_iter; ++iter) // 温度迭代
    {
        for (int k = 0; k < Lk; ++k) // 每个温度下的迭代
        {
            // 生成可行的新路径
            float new_cost = NewPath(current_path, new_path);
            if (new_cost < 0.0f) {
                cout << "No suitable new path in this condition!" << endl;
                return;
            }

            // 根据Metropolis准则计算一个概率更新解
            float p_update = this->P();

            if (new_cost < current_cost || p_update < exp(-(new_cost - current_cost) / Tc)) 
            {
                current_path = new_path;
                current_cost = new_cost;
            }
            
            // 记录最优解
            if (current_cost < best_cost)
            {
                rectify_flag = 0;
                best_path = current_path;
                best_cost = current_cost;
            }
        }

        rectify_flag++;
        if (rectify_flag == 5)
        {
            rectify_flag = 0;
            current_path = best_path;
            current_cost = best_cost;
        }
        // 衰减温度
        Tc = Tc * alpha;
    }
    this->SplitPath(best_path);
}

// 初始化运输路径
float SA::InitPath(vector<int> &init_path)
{
    int taskNum = this->costs.size().width;
    int nodeNum = this->costs.size().height;
    vector<int> task_index(nodeNum, -1);
    for (int i = 0; i < nodeNum; ++i)
    {
        task_index.at(i) = i;
    }

    int break_cnt = 0;
    float total_cost = -1.0f;
    do {
#if 1
        Fisher_yates_Shuffle(task_index, init_path);
#else
        init_path = this->task_index;
        Knuth_Durstenfeld_Shuffle(init_path);
#endif
        total_cost = this->CalculateCost(init_path);
    }
    while (total_cost < 0.0f || break_cnt++ < 100);
    return total_cost;
}

// 模拟退火算法生成新解
float SA::NewPath(vector<int> old_path, vector<int>&new_path)
{
    new_path.clear();

    uniform_int_distribution<> dis(0, old_path.size());

    int break_cnt = 0;
    float total_cost = -1.0f;
    do {
        int id1, id2, id3;
        do {
            id1 = dis(gen);
            id2 = dis(gen);
            id3 = dis(gen);
        } while (id1 == id2 || id1 == id3 || id2 == id3);
        std::vector<int> idx = {id1, id2, id3};
        std::sort(idx.begin(), idx.end());
        float p = this->P();

        // 三种生成新解的方式
        if (p < 0.25)
        {
            // 0.25的概率交换法: AbCdE ->AdCbE
            std::swap(old_path.at(id1), old_path.at(id2));
            new_path = old_path;
        }
        else if (p < 0.5)
        {
            // 0.25的概率移位法: ABCD -> ACBD
            new_path.insert(new_path.end(), old_path.begin(), old_path.begin() + idx[0]);
            new_path.insert(new_path.end(), old_path.begin() + idx[1], old_path.begin() + idx[2]);
            new_path.insert(new_path.end(), old_path.begin() + idx[0], old_path.begin() + idx[1]);
            new_path.insert(new_path.end(), old_path.begin() + idx[2], old_path.end());
        }
        else if (p < 0.75)
        {
            // 0.25的概率移位法: ABCD -> ACBD
            new_path.insert(new_path.end(), old_path.begin(), old_path.begin() + idx[0]);
            new_path.insert(new_path.end(), old_path.begin() + idx[1] + 1, old_path.begin() + idx[2] + 1);
            new_path.insert(new_path.end(), old_path.begin() + idx[0], old_path.begin() + idx[1] + 1);
            new_path.insert(new_path.end(), old_path.begin() + idx[2] + 1, old_path.end());
        }
        else
        {
            // 0.25的概率倒置法: Abcd...wxyZ -> Ayxw...dcbZ
            reverse(old_path.begin() + id1, old_path.begin() + id2);
            new_path = old_path;
        }
        total_cost = this->CalculateCost(new_path);
    } while (total_cost < 0.0f || break_cnt++ < 100);
    return total_cost;
}

// 分割路径
void SA::SplitPath(std::vector<int> &path) 
{
    this->splited_paths.clear();
    // 找到分割点
    vector<int>::iterator beg = path.begin();
    vector<int>::iterator end = path.begin();

    for (vector<int>::iterator i = path.begin(); i != path.end(); ++i)
    {
        if (-1 == *i)
        {
            end = i;
            this->splited_paths.push_back(vector<int>(beg, end));
            beg = end;
        }
    }
    this->splited_paths.push_back(vector<int>(end, path.end()));
}

// 计算分割好的路径的总代价
float SA::CalculateCost(std::vector<int> &path) {
    this->SplitPath(path);

    float total_cost = 0;
    for (int i = 0; i < this->carNum; ++i) {
        float cost = 0;
        int car_index = i + this->taskNum;
        const vector<int> &p = this->splited_paths.at(i);

        if (p.size() > 0)
        {
            float per_cost = this->costs.at<float>(car_index, p.at(0));
            if (per_cost < 0) { return -1.0f; }

            cost += per_cost;
            for (int j = 0; j < p.size() - 1; ++j) {
                // Check if sensor is ok for this task
                per_cost = this->costs.at<float>(car_index, p.at(j + 1));
                if (per_cost < 0.0f) { return -1.0f; }

                per_cost = this->costs.at<float>(p.at(j), p.at(j + 1));
                if (per_cost < 0.0f) {
                    return -1.0f; 
                }
                cost += per_cost;
            }
        }
        total_cost += cost;
    }
    return total_cost;
}
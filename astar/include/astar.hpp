#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
using Mat = cv::Mat;

struct Node
{
    int x;
    int y;
    float f;
    float g;
    float h;
    Node *parent;

    Node(int x_, int y_, float g_, float h_, Node *parent_) : x(x_), y(y_), f(g_ + h_), g(g_), h(h_), parent(parent_) {}

    bool operator<(const Node &other) const
    {
        return f > other.f;
    }
};

std::pair<float, std::vector<cv::Point>> a_star(cv::Mat &map, cv::Point start, cv::Point goal);

float CalculateA(cv::Mat &map, float x0, float y0, float x1, float y1);

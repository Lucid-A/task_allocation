#include <queue>
#include "astar.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using Mat = cv::Mat;

pair<float, vector<cv::Point>> a_star(Mat &map, cv::Point start, cv::Point goal)
{
    priority_queue<Node> open_list;
    vector<vector<bool>> closed_list(map.rows, vector<bool>(map.cols, false));

    open_list.push(
        Node(
            start.x, start.y,
            0, abs(goal.x - start.x) + abs(goal.y - start.y),
            nullptr));

    while (!open_list.empty())
    {
        Node current = open_list.top();
        open_list.pop();

        if (current.x == goal.x && current.y == goal.y)
        {
            vector<cv::Point> path;
            float cost = current.f;
            while (current.parent != nullptr)
            {
                path.push_back(cv::Point(current.x, current.y));
                current = *current.parent;
            }
            return pair<float, vector<cv::Point>>(cost, path);
        }

        if (closed_list[current.y][current.x])
            continue;

        closed_list[current.y][current.x] = true;

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                int nx = current.x + dx;
                int ny = current.y + dy;

                if (nx < 0 || nx >= map.cols || ny < 0 || ny >= map.rows)
                    continue;

                if (map.at<uchar>(ny, nx) == 255)
                    continue;

                if (closed_list[ny][nx])
                    continue;

                float g = current.g + sqrt(dx * dx + dy * dy);
                float h = abs(goal.x - nx) + abs(goal.y - ny);

                open_list.push(Node(nx, ny, g, h,
                                    new Node(current)));
            }
        }
    }
    return {};
}

float CalculateA(Mat &map, float x0, float y0, float x1, float y1)
{
    auto ret = a_star(map, cv::Point(x0, y0), cv::Point(x1, y1));
    return ret.first;
}
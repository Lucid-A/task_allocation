#include <utility>
#include <queue>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using Mat = cv::Mat;

const string map_file = "/home/nx/astar/MapData/test_map.pgm";
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

pair<float, vector<cv::Point>> a_star(Mat &map, cv::Point start, cv::Point goal)
{
#ifdef SHOW
    Mat show, show_big;
    map.copyTo(show);
#endif
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
#ifdef SHOW
        cv::circle(show,
                   cv::Point(current.x, current.y),
                   1, cv::Scalar(127), -1);

        cv::waitKey(10);

        cv::resize(show, show_big, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);
        cv::imshow("A*", show_big);
#endif
    }

    return {};
}

cv::Point start(-1, -1);
cv::Point end_point(-1, -1);

void on_mouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (start.x < 0 && start.y < 0)
        {
            start = cv::Point(x, y);
            cout << "start: (" << start << ")" << endl;
        }
        else if (end_point.x < 0 && end_point.y < 0)
        {
            end_point = cv::Point(x, y);
            cout << "end: (" << end_point << ")" << endl;
        }
    }
}

int main()
{

    cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);

    cv::namedWindow("A*", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("A*", 100, 100);
    cv::waitKey(10);

    cv::setMouseCallback("Map", on_mouse, NULL);
    cv::waitKey(10);

    // Read Map file
    Mat map = cv::imread(map_file, cv::IMREAD_GRAYSCALE);

    if (!map.data)
    {
        printf("No image data \n");
        return -1;
    }

    // Make it Black and White
    Mat binary;
    double thresh = 230;
    double maxval = 255;
    cv::threshold(map, binary, thresh, maxval, cv::THRESH_BINARY);

    // Make Obstacle White(255) and Road Black(0)
    cv::bitwise_not(binary, binary);

    // Do obstacle expansion
    Mat expand;
    int dilation_size = 6;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * dilation_size, 2 * dilation_size),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(binary, expand, element, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    expand.copyTo(map);

    Mat show;
    binary.copyTo(show);
    Mat show_big;

    cv::imshow("Map", map);

    while (1)
    {
        if (start.x >= 0 && start.y >= 0)
        {
            cv::circle(show, start, 1, cv::Scalar(200), -1);

            if (end_point.x >= 0 && end_point.y >= 0)
            {
                cv::circle(show, end_point, 1, cv::Scalar(60), -1);
                break;
            }
        }

        cv::resize(show, show_big, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);
        cv::imshow("A*", show_big);

        char c = cv::waitKey(33);

        if (c == 27)
        {
            break;
        }
    }

    cout << "start a*" << endl;
    // Get the start time
    auto start_time = std::chrono::high_resolution_clock::now();

    float cost;
    std::vector<cv::Point> path;
    auto ret = a_star(expand, start, end_point);
    cost = ret.first;
    path = ret.second;

    // Get the end time
    auto end = std::chrono::high_resolution_clock::now();
    cout << "end a*" << endl;

    // Calculate the duration in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_time).count();
    std::cout << "Execution time: " << duration / 1000.0 << "ms" << std::endl;
    std::cout << "Path cost: " << cost << std::endl;

    cout << path.size() << endl;
    for (int i = 0; i < path.size(); i++)
    {
        cv::circle(show, cv::Point(path[i].x, path[i].y), 1, cv::Scalar(127), -1);

        cv::resize(show, show_big, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);
        cv::imshow("A*", show_big);

        cv::waitKey(33);
    }
    cv::waitKey(0);

    return 0;
}
#if 0
int main()
{
    // Read Map file
    Mat map = cv::imread(map_file, cv::IMREAD_GRAYSCALE);

    if (!map.data)
    {
        printf("No image data \n");
        return -1;
    }

    cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Map", map);

    // Make it Black and White
    Mat binary;
    double thresh = 230;
    double maxval = 255;
    cv::threshold(map, binary, thresh, maxval, cv::THRESH_BINARY);

    // Make Obstacle White(255) and Road Black(0)
    cv::bitwise_not(binary, binary);

    cv::namedWindow("Binary Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Binary Map", binary);

    // Do obstacle expansion
    Mat expand;
    int dilation_size = 6;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * dilation_size, 2 * dilation_size),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(binary, expand, element, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

    // Resize Expanded Map for visualize
    cv::resize(expand, expand, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);

    cv::namedWindow("Expand Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Expand Map", expand);

    int key = cv::waitKey(0);

    return 0;
}
#endif

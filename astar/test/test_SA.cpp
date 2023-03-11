#include "SA.h"
#include "astar.hpp"

const std::string map_file = "/home/nx/astar/MapData/test_map.pgm";

void on_mouse(int event, int x, int y, int flag, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::Point *p = (cv::Point *)userdata;
        p->x = x;
        p->y = y;
    }
}

int main()
{
    Task task[6];

    Car car[3];

    cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
    cv::Point p;
    cv::setMouseCallback("Map", on_mouse, &p);
    cv::waitKey(10);

    cv::namedWindow("A*", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("A*", 100, 100);
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

    cv::resize(show, show_big, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);
    cv::imshow("A*", show_big);
    cv::imshow("Map", map);

    std::cout << "Please Click Mouse Left key to choose a point" << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        std::cout << "Car" << i << " Initial Position: ";
        p = cv::Point(-1, -1);
        while (1)
        {

            if (p.x >= 0 && p.y >= 0)
            {
                car[i] = {i, float(p.x), float(p.y), {1, 1}};

                std::cout << p << std::endl;

                cv::circle(map, p, 2, cv::Scalar(255), -1);
                cv::circle(show, p, 2, cv::Scalar(255), -1);
                cv::resize(show, show_big, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);
                cv::imshow("A*", show_big);
                cv::waitKey(23);

                break;
            }

            char c = cv::waitKey(23);
        }
    }

    for (int i = 0; i < 6; ++i)
    {
        std::cout << "Task" << i << " Type: " << (i < 3 ? 'A' : 'B') << " Position: ";
        p = cv::Point(-1, -1);
        while (1)
        {

            if (p.x >= 0 && p.y >= 0)
            {
                task[i] = {i, float(p.x), float(p.y), i < 3 ? 'A' : 'B'};

                std::cout << p << std::endl;

                cv::circle(show, p, 1, i < 3 ? cv::Scalar(200) : cv::Scalar(140), -1);
                cv::resize(show, show_big, cv::Size(), 8.0, 8.0, cv::INTER_NEAREST);
                cv::imshow("A*", show_big);

                cv::waitKey(23);

                break;
            }

            cv::waitKey(23);
        }
    }

    std::cout << "start SA" << std::endl;
    // Get the start time
    auto start_time = std::chrono::high_resolution_clock::now();

    SA sa(MINTIME);
    sa.StartSA(map, 3, car, 6, task);

    // Get the end time
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "end SA" << std::endl;

    // Calculate the duration in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time).count();
    std::cout << "Execution time: " << duration / 1000.0 << "s" << std::endl;

    std::cout << "best_path:";
    for (int i = 0; i < sa.best_path.size(); i++)
        std::cout << sa.best_path[i] << ' ';
    std::cout << std::endl;
    std::cout << "best_cost:" << sa.best_cost << std::endl;
    std::cout << "each car path:" << std::endl;

    for (int i = 0; i < car[0].path.size(); i++)
        std::cout << car[0].path[i].TaskNum << ' ';
    std::cout << std::endl;

    for (int i = 0; i < car[1].path.size(); i++)
        std::cout << car[1].path[i].TaskNum << ' ';
    std::cout << std::endl;

    for (int i = 0; i < car[2].path.size(); i++)
        std::cout << car[2].path[i].TaskNum << ' ';

    return 0;
}

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <set>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const string map_file = "./MapData/xunjian_new.pgm";

int main()
{
    Mat image = imread(map_file, IMREAD_GRAYSCALE);

    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }

    Mat resized;
    cout << image.size << endl;
    resize(image, resized, Size(), 8.0, 8.0);

#if 0
    int type = image.type();
    int depth = CV_MAT_DEPTH(type);

    switch (depth)
    {
    case CV_8U:
        cout << "Data type: 8-bit unsigned integer" << endl;
        break;
    case CV_8S:
        cout << "Data type: 8-bit signed integer" << endl;
        break;
    case CV_16U:
        cout << "Data type: 16-bit unsigned integer" << endl;
        break;
    case CV_16S:
        cout << "Data type: 16-bit signed integer" << endl;
        break;
    case CV_32S:
        cout << "Data type: 32-bit signed integer" << endl;
        break;
    case CV_32F:
        cout << "Data type: 32-bit floating point" << endl;
        break;
    case CV_64F:
        cout << "Data type: 64-bit floating point" << endl;
        break;
    default:
        cout << "Unknown data type" << endl;
    }
#endif

    // Calculate histogram
    int histSize = 256;
    float range[] = {0, 256};
    const float *histRange = {range};
    Mat hist;
    cout << image.channels() << endl;
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, &histRange);

    // Print histogram values
    // for (int i = 0; i < histSize; i++)
    //     cout << "Value " << i << " = " << hist.at<float>(i) << endl;

    // Create a binary image
    Mat binary;
    double thresh = 127;
    double maxval = 255;
    threshold(image, binary, thresh, maxval, THRESH_BINARY);

    // Create a ternary image with 0 127 255
    Mat ternary = Mat::zeros(image.size(), CV_8UC1);
    Mat mask;
    inRange(image, 0, 170, mask);
    ternary.setTo(127, mask);
    inRange(image, 86, 170, mask);
    ternary.setTo(255, mask);

    cout << int(image.at<uint8_t>(0, 0)) << endl;

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);

    namedWindow("Resized Image", WINDOW_AUTOSIZE);
    imshow("Resized Image", resized);

    namedWindow("Binary Image", WINDOW_AUTOSIZE);
    imshow("Binary Image", binary);

    namedWindow("Ternary Image", WINDOW_AUTOSIZE);
    imshow("Ternary Image", ternary);

    waitKey(0);

    return 0;
}

#include "bresenham_test.h"

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: <bin_name> <image_name> " << std::endl;
        return -1;
    }
    std::string image_name(argv[1]);
    cv::Mat img = cv::imread(image_name);
    cv::imshow("image original", img);
    
    cv::cvtColor(img, img, CV_RGB2GRAY);
    // void draw_line(cv::Mat& img, int x1, int y1, int x2, int y2, unsigned long c);  // 在img图像中，以(x1, y1)为起点，（x2, y2）为终点（包括终点）画线段, 划线的像素值为c;
    draw_line(img, 500, 500, 100, 100, 0);
    // void draw_circle(cv::Mat& img, int xc, int yc, int r, unsigned long c, bool is_fill = 0);  // 在img图像中，以(xc, yc)为起点，r为半径画圆，划线的像素值为c，若is_fill为false，不填充（默认值），否则填充;
    draw_circle(img, 100, 100, 20, 0, 1);
    std::cout << img.rows << ", " << img.cols << std::endl;
    
    cv::imshow("after plot", img);
    cv::waitKey(0);
    
    return 0;
}

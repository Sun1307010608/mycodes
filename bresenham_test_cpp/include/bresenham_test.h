#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

void swap_int(int& a, int& b)
{
    int t = a;
    a = b;
    b = t;
    return ;
}

void putpixel(cv::Mat& img, int cx, int cy, unsigned long c)
{
    if (cx < 0 || cx >= img.rows || cy < 0 || cy>= img.cols)
    {
        std::cout << "point ( "<< cx << ", " << cy << " ) is not inside the image, continue! " << std::endl;
        return ;
    }
    if (img.channels() == 1)
    {
        img.at<uchar>(cx, cy) = c;
    }
    else if (img.channels() >= 3) {
        img.at<cv::Vec3b>(cx, cy)[0] = c;
        img.at<cv::Vec3b>(cx, cy)[1] = c;
        img.at<cv::Vec3b>(cx, cy)[2] = c;
    }
    else {
        std::cout << "the image channels ... " << std::endl;
    }
    return ;
}

// 在img图像中，以(x1, y1)为起点，（x2, y2）为终点（包括终点）画线段, 划线的像素值为c;
// Bresenham's line algorithm
void draw_line(cv::Mat& img, int x1, int y1, int x2, int y2, unsigned long c) 
{
    if (x1 < 0 || x1 >= img.rows || y1 < 0 || y1 >= img.cols)
        std::cout << "start point (" << x1 << ", " << y1 << ") is not inside the image, exit!" << std::endl;
    if (x2 < 0 || x2 >= img.rows || y2 < 0 || y2 >= img.cols)
        std::cout << "start point (" << x2 << ", " << y2 << ") is not inside the image, exit!" << std::endl;
	// 参数 c 为颜色值
	int dx = abs(x2 - x1),
		dy = abs(y2 - y1),
		yy = 0;

	if (dx < dy) 
	{
		yy = 1;
		swap_int(x1, y1);
		swap_int(x2, y2);
		swap_int(dx, dy);
	}

	int ix = (x2 - x1) > 0 ? 1 : -1,
		iy = (y2 - y1) > 0 ? 1 : -1,
		cx = x1,
		cy = y1,
		n2dy = dy * 2,
		n2dydx = (dy - dx) * 2,
		d = dy * 2 - dx;

	if (yy) 
	{ // 如果直线与 x 轴的夹角大于 45 度
		while (cx != x2) 
		{
			if (d < 0) {
				d += n2dy;
			} else {
				cy += iy;
				d += n2dydx;
			}
			putpixel(img, cy, cx, c);
			cx += ix;
		}
	} else { // 如果直线与 x 轴的夹角小于 45 度
		while (cx != x2) 
		{
			if (d < 0) {
				d += n2dy;
			} else {
				cy += iy;
				d += n2dydx;
			}
			putpixel(img, cx, cy, c);
			cx += ix;
		}
	}
}

inline void draw_circle_8(cv::Mat& img, int xc, int yc, int x, int y, unsigned long c) {
	// 参数 c 为颜色值
	putpixel(img, xc + x, yc + y, c);
	putpixel(img, xc - x, yc + y, c);
	putpixel(img, xc + x, yc - y, c);
	putpixel(img, xc - x, yc - y, c);
	putpixel(img, xc + y, yc + x, c);
	putpixel(img, xc - y, yc + x, c);
	putpixel(img, xc + y, yc - x, c);
	putpixel(img, xc - y, yc - x, c);
}

//Bresenham's circle algorithm
// 在img图像中，以(xc, yc)为起点，r为半径画圆，划线的像素值为c，若is_fill为false，不填充（默认值），否则填充;
void draw_circle(cv::Mat& img, int xc, int yc, int r, unsigned long c, bool is_fill = 0) 
{
	// (xc, yc) 为圆心，r 为半径
	// is_fill 为是否填充
	// c 为颜色值

	// 如果圆在图片可见区域外，直接退出
	if (xc + r < 0 || xc - r >= img.cols ||	yc + r < 0 || yc - r >= img.rows) 
	{
	    std::cout << "the circle is not completely inside the image, exit! " << std::endl;
	    return;
	}

	int x = 0, y = r, yi, d;
	d = 3 - 2 * r;

	if (is_fill) 
	{
		// 如果填充（画实心圆）
		while (x <= y) 
		{
			for (yi = x; yi <= y; yi ++)
				draw_circle_8(img, xc, yc, x, yi, c);

			if (d < 0) 
			{
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y --;
			}
			x++;
		}
	} else {
		// 如果不填充（画空心圆）
		while (x <= y) 
		{
			draw_circle_8(img, xc, yc, x, y, c);

			if (d < 0) 
			{
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y --;
			}
			x ++;
		}
	}
}

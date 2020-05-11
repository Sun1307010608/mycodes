#include <iostream>
#include <string>

#include <nav_msgs/OccupancyGrid.h>


void swap_int(int& a, int& b)
{
    int t = a;
    a = b;
    b = t;
    return ;
}

void set_logit(std::vector<double>& logits, int cx, int cy, double c, const int grid_width, const int grid_height)
{
    if (cx < 0 || cx >= grid_height || cy < 0 || cy >= grid_width)
        return ;
    logits[cy*grid_height+cx] += c;
}

// Bresenham's line algorithm, 在map图像中，以(x1, y1)为起点，（x2, y2）为终点画线段, 划线的像素值为c;
void draw_line(std::vector<double>& logits, int x1, int y1, int x2, int y2, double c, const int grid_width, const int grid_height) 
{
    //size_t rows = map.info.height, cols = map.info.width;
    //if (x1 < 0 || x1 >= rows || y1 < 0 || y1 >= cols)
      //  std::cout << "start point (" << x1 << ", " << y1 << ") is not inside the image, exit!" << std::endl;
    //if (x2 < 0 || x2 >= rows || y2 < 0 || y2 >= cols)
      //  std::cout << "start point (" << x2 << ", " << y2 << ") is not inside the image, exit!" << std::endl;
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
			set_logit(logits, cy, cx, c, grid_width, grid_height);
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
			set_logit(logits, cx, cy, c, grid_width, grid_height);
			cx += ix;
		}
	}
}

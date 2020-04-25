function [img] = bresenham_line(img, x1, y1, x2, y2, value)
%bresenham_line(img, x1, y1, x2, y2, value), 在图像img上以（x1, y1）为起点，
%(x2, y2)为终点画直线，value表示画线的像素值。

dx = abs(x2 - x1);
dy = abs(y2 - y1);
yy = 0;
if dx < dy
    yy = 1;
    [x1, y1] = swap(x1, y1);
    [x2, y2] = swap(x2, y2);
    [dx, dy] = swap(dx, dy);
end

if x2 - x1 > 0
    ix = 1;
else
    ix = -1;
end
if y2 - y1 > 0
    iy = 1;
else
    iy = -1;
end

cx = x1;
cy = y1;
n2dy = dy*2;
n2dydx = (dy - dx) * 2;
d = dy*2 - dx;

if yy == 1
    while cx ~= x2
        if d < 0
            d = d+ n2dy;
        else
            cy = cy + iy;
            d = d + n2dydx;
        end
        img(cy, cx) = value;
        cx = cx + ix;
    end
else
    while cx ~= x2
        if d < 0
            d = d+ n2dy;
        else
            cy = cy + iy;
            d = d + n2dydx;
        end
        img(cx, cy) = value;
        cx = cx + ix;
    end
end
end

function [a, b] = swap(a, b)
t = a;
a = b;
b = t;
end


function [img] = bresenham_circle(img, xc, yc, r, value, is_fill)
%bresenham_circle(img, xc, yc, r, is_fill, value), 在图像img上，以(xc,yc)为圆心，
%r为半径画圆, is_fill为0(默认值)，表示不填充，否则填充整个圆; value表示画圆像素值。

if nargin < 6
    is_fill = 0;
end

[h, w] = size(img);
if xc+r < 0 || xc-r >= w || yc+r < 0 || yc-r >= h
    return ;
end
x = 0;
y = r;
d = 3 - 2*r;
if is_fill ~= 0
    while x <= y
        for yi = x : y
            img = draw_circle(img, xc, yc, x, yi, value);
        end
        if d < 0
            d = d + 4*x + 6;
        else
            d = d + 4*(x-y) + 10;
            y = y-1;
        end
        x = x+1;
    end
else
    while x <= y
        img = draw_circle(img, xc, yc, x, y, value);
        if d < 0
            d = d + 4*x + 6;
        else
            d = d + 4*(x-y) + 10;
            y = y-1;
        end
        x = x+1;
    end
end
end

function img = draw_circle(img, xc, yc, x, y, value)
img(xc+x, yc+y) = value;
img(xc-x, yc+y) = value;
img(xc+x, yc-y) = value;
img(xc-x, yc-y) = value;
img(xc+y, yc+x) = value;
img(xc-y, yc+x) = value;
img(xc+y, yc-x) = value;
img(xc-y, yc-x) = value;
end
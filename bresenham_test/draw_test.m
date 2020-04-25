close all; clear all; clc

% create image reactangle:
img = ones(100, 200);
figure('Name', 'original image')
imshow(img)

% draw line:
help bresenham_line
draw_value = 0;
x1 = 80; y1 = 20;
x2 = 20; y2 = 60;
img = bresenham_line(img, x1, y1, x2, y2, draw_value);

% draw circle:
help bresenham_circle
cx = 50; cy = 50;
r = 30;
img = bresenham_circle(img, cx, cy, r, draw_value);
figure('Name', 'after drawn')
imshow(img)
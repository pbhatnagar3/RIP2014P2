% Create a purple transparent circle
xc = 3.0;
yc = 1.0;
r = 0.1;
x = r*sin(-pi:0.1*pi:pi) + xc;
y = r*cos(-pi:0.1*pi:pi) + yc;
c = [0.6 0 1];
fill(x, y, c, 'FaceAlpha', 0.4)
axis square
axis equal
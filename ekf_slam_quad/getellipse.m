function [x,y,z] = getellipse(C, x, y)

n=100; % Number of points around ellipse
p=0:pi/n:2*pi; % angles around a circle

[eigvec,eigval] = eig(C); % Compute eigen-stuff
xy = [cos(p'),sin(p')] * sqrt(eigval) * eigvec'; % Transformation
x = xy(:,1) + x;
y = xy(:,2) + y;
z = zeros(size(x));


end
function [xx,yy,zz]=error_ellipse_2d(C,mu,scale)
%    ERROR_ELLIPSE(C,MU) - Plot the ellipse, or ellipsoid, centered at MU,
%    a vector whose length should match that of C (which is 2x2 or 3x3).
%    NOTES: C must be positive definite for this function to work properly.

[r,c] = size(C);

x0=mu(1);
y0=mu(2);
z0=mu(3);

k = scale;
 
% Make sure the matrix has positive eigenvalues - else it's not a valid covariance matrix!
if any(eig(C) <=0)
error('The covariance matrix must be positive definite (it has non-positive eigenvalues)')
end

% C is 3x3; extract the 2x2 matricies, and plot the associated error
% ellipses. They are drawn in space, around the ellipsoid; it may be
% preferable to draw them on the axes.
Cxy = C(1:2,1:2);
% Cyz = C(2:3,2:3);
% Czx = C([3 1],[3 1]);

[x,y,z] = getpoints(Cxy);
x_xy = x0+k*x;
y_xy = y0+k*y;
z_xy = z0+k*z;
[y,z,x] = getpoints([1 0; 0 1]);
x_yz = x0+k*x;
y_yz = y0+k*y;
z_yz = z0+k*z;
[z,x,y] = getpoints([1 0;0 1]);
x_zx = x0+k*x;
y_zx = y0+k*y;
z_zx = z0+k*z;
  
xx = [x_xy;NaN;x_yz;NaN;x_zx];
yy = [y_xy;NaN;y_yz;NaN;y_zx];
zz = [z_xy;NaN;z_yz;NaN;z_zx];

  
%   [eigvec,eigval] = eig(C);
% 
%   [X,Y,-Z] = ellipsoid(0,0,0,1,1,1);
%   XYZ = [X(:),Y(:),Z(:)]*sqrt(eigval)*eigvec';
%   
%   X(:) = (k*XYZ(:,1)+x0);
%   Y(:) = (k*XYZ(:,2)+y0);
%   Z(:) = (k*XYZ(:,3)+z0);
%   h4=surf(X,Y,Z);
%   colormap gray
%   alpha(0.3)
%   camlight
end
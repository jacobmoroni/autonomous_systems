function h=error_ellipse_2d(C,mu,scale)
% ERROR_ELLIPSE - plot an error ellipse, or ellipsoid, defining confidence region
%    ERROR_ELLIPSE(C22) - Given a 2x2 covariance matrix, plot the
%    associated error ellipse, at the origin. It returns a graphics handle
%    of the ellipse that was drawn.
%
%    ERROR_ELLIPSE(C33) - Given a 3x3 covariance matrix, plot the
%    associated error ellipsoid, at the origin, as well as its projections
%    onto the three axes. Returns a vector of 4 graphics handles, for the
%    three ellipses (in the X-Y, Y-Z, and Z-X planes, respectively) and for
%    the ellipsoid.
%
%    ERROR_ELLIPSE(C,MU) - Plot the ellipse, or ellipsoid, centered at MU,
%    a vector whose length should match that of C (which is 2x2 or 3x3).
%
%    ERROR_ELLIPSE(...,'Property1',Value1,'Name2',Value2,...) sets the
%    values of specified properties, including:
%      'C' - Alternate method of specifying the covariance matrix
%      'mu' - Alternate method of specifying the ellipse (-oid) center
%      'conf' - A value betwen 0 and 1 specifying the confidence interval.
%        the default is 0.5 which is the 50% error ellipse.
%      'scale' - Allow the plot the be scaled to difference units.
%      'style' - A plotting style used to format ellipses.
%      'clip' - specifies a clipping radius. Portions of the ellipse, -oid,
%        outside the radius will not be shown.
%
%    NOTES: C must be positive definite for this function to work properly.

[r,c] = size(C);

x0=mu(1);
y0=mu(2);
z0=mu(3);

k = scale;
 
% Make the matrix has positive eigenvalues - else it's not a valid covariance matrix!
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
h1=plot3(x0+k*x,y0+k*y,z0+k*z);hold on
[y,z,x] = getpoints(Cyz);
h2=plot3(x0+k*x,y0+k*y,z0+k*z);hold on
[z,x,y] = getpoints(Czx);
h3=plot3(x0+k*x,y0+k*y,z0+k*z);hold on
  


  
  [eigvec,eigval] = eig(C);

  [X,Y,Z] = ellipsoid(0,0,0,1,1,1);
  XYZ = [X(:),Y(:),Z(:)]*sqrt(eigval)*eigvec';
  
  X(:) = (k*XYZ(:,1)+x0);
  Y(:) = (k*XYZ(:,2)+y0);
  Z(:) = (k*XYZ(:,3)+z0);
  h4=surf(X,Y,Z,'r');
  colormap gray
  alpha(0.3)
  camlight
  if nargout
    h=[h1 h2 h3 h4];
  end
end
% %axis equal
% 
% set(gca,'nextplot',hold_state);
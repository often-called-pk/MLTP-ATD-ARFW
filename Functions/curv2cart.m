function [x, y] = curv2cart(s,k,o,theta)
% Reconstruct track (s,k) -> (x,y), get cartesian coordinates from curvilinear coordinates
%   ·s = cumulative distance along curve (m)
%   ·k = curvature, with sign (m^-1)
%   ·o = 1 or -1. By default it is assumed that if k>0 it is a left-hand turn,
%    this can be reversed by passing 'o' with a value of -1
%   ·theta = orientation of the first segment in the cartesian plane (deg). Zero by default

thr = 1e-4; %threshold for the curvature to be considered as a straight

if nargin == 4
    if o == -1
        k = -k;
    end
elseif nargin == 3
    if o == -1
        k = -k;
    end
    theta = 0;
elseif nargin == 2
    theta = 0;
elseif nargin ~= 2
    error("Check number of inputs");
end

s = s(:)';

N = size(s(:),1);
ds = [0 diff(s)];

X = zeros(N,2);
X(1,:) = [0 0];
X(2,:) = rotatePoint2D(theta,[s(2) 0]);

for i = 3:N
    if abs(k(i-1)) < thr %straights
        X(i,:) = X(i-1,:) + (X(i-1,:)-X(i-2,:))/norm(X(i-1,:)-X(i-2,:))*ds(i);
    else
        a = asin(k(i-1)*ds(i-1)/2) + asin(k(i-1)*ds(i)/2);
        b = atan2(X(i-1,2)-X(i-2,2), X(i-1,1)-X(i-2,1));
        [u, v] = pol2cart(b+a, ds(i));
        X(i,:) = X(i-1,:) + [u v];
    end
x = X(:,1);
y = X(:,2);

end
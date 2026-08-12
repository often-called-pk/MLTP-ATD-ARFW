function [x,y] = cartPath(x0,y0,n)
%Retrieve cartesian coordinates of the vehicle trajectory from the centre line
% coordinates (x0,y0) and the normal distance to the centre line (n)

X0 = [x0(:) y0(:)];
dx = diff(X0);

n_vec = [-dx(:,2) dx(:,1)]./vecnorm(dx')';
X = X0 + n(:).*[n_vec; n_vec(end,:)];

x = X(:,1);
y = X(:,2);

end
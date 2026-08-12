function [Xl, Xr] = trackLimits(x0,y0,w)
%Find cartesian coordinates of the left and right track limits from the centre line and width of the track

X0 = [x0(:) y0(:)];
dx = diff(X0);

n_vec = [-dx(:,2) dx(:,1)]./vecnorm(dx')';
Xl = X0(1:end-1,:) + abs(w(:)/2).*n_vec;
Xr = X0(1:end-1,:) - abs(w(:)/2).*n_vec;

Xl(end+1,:) = 2*Xl(end,:)-Xl(end-1,:);
Xr(end+1,:) = 2*Xr(end,:)-Xr(end-1,:);

end
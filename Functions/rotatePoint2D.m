function rotatedPoint = rotatePoint2D(alpha, P)
%rotate point P around axis u passing through the origin
P = reshape(P,1,[]);
%alpha in degrees.
if numel(alpha) ~= 1
    error('Angle of rotation must be a scalar.');
end

s = sin(alpha);
c = cos(alpha);

% 2D rotation matrix:
R = [c, -s;  s, c];

rotatedPoint = P * R; 
end
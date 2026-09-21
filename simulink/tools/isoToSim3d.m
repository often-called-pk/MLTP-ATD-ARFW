function [tEng, rEng] = isoToSim3d(tIso, rIso)
%ISOTOSIM3D  ISO 8855 pose -> the sim3d ENGINE frame a 3D block expects.
%   [tEng, rEng] = ISOTOSIM3D(tIso, rIso)
%
%   tIso  1x3 [x y z]           ISO 8855 metres  (x forward, y LEFT, z up)
%   rIso  1x3 [roll pitch yaw]  ISO 8855 RADIANS (yaw positive counter-clockwise)
%
%   tEng  1x3 metres, sim3d engine frame (y right)
%   rEng  1x3 DEGREES, sim3d engine frame
%
%   Why this exists. Most sim3d actors expose a CoordinateSystem property
%   that can simply be set to 'ISO8855', after which the pose goes in
%   unconverted. Two things in this project cannot do that:
%
%     * the Vehicle Dynamics Blockset "Simulation 3D Vehicle" block, whose
%       mask has NO CoordinateSystem parameter at any level (checked), so its
%       Translation/Rotation ports are in the engine frame; and
%     * anything driven from a MATLAB Function block, which has no actor
%       handle to set the property on.
%
%   The mapping is NOT guessed. It is exactly what MathWorks' own converter
%   sim3d.utils.TransformISO8855 produces -- convert() for the translation
%   and convertRotation() for the rotation -- and
%   simulink/validation/test_unrealPose.m asserts that equality over a grid
%   of random poses, so a future release changing the convention fails the
%   test instead of silently mis-posing the car.
%
%       translation  [ x, -y,  z ]                metres
%       rotation     [ roll, -pitch, -yaw ]       converted to DEGREES
%
%   Pure elementwise arithmetic, no branch and no toolbox call, so it is
%   valid inside a MATLAB Function block and in plain MATLAB alike.
%
%   See also unrealPlayback, buildTrackRibbon.

%#codegen

tEng = [ tIso(1), -tIso(2),  tIso(3) ];
rEng = [ rIso(1), -rIso(2), -rIso(3) ] * (180/pi);
end

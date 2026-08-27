function nu = nuForConfig(ATD, ActAero, rwMandate)
%NUFORCONFIG Number of control inputs for a (pt.ATD, vp.ActAero) combination.
%
%   nu = nuForConfig(pt.ATD, vp.ActAero)
%   nu = nuForConfig(pt.ATD, vp.ActAero, vp.rwMandate)
%
%   Scripts/vehModel.m holds the authoritative ladder; this mirrors it. It has to
%   exist because the warm-start lookup in the entry points runs BEFORE vehModel,
%   so nu is not yet defined there - and without nu, latestInit cannot tell an AWD
%   cache (nu=4) from an ATD one (nu=8) at the same aeroSetting, which is exactly
%   how an ATD run ends up warm-starting from an AWD lap.
%
%   Any change to the input set must be applied to every branch of every ladder,
%   this table included.
%
%       ATD  ActAero  nu     controls
%       ---  -------  --     ---------------------------------------------
%        0      0       3    T_motor, T_brake, delta
%        0      1       4    + activeAeroRW                (wing 2nd-to-last)
%        0      2       5    + activeAeroFW
%        0      3       7    + FL/FR/TW
%        1      0       7    T_motor, T_brake, ATD x4, delta
%        1      1       8    + activeAeroRW                (wing 2nd-to-last)
%        1      2       9    + activeAeroFW
%        1      3      11    + FL/FR/TW
%
%   rwMandate is optional (default 0). Only modes 6 and 7 change nu: both add
%   activeAeroFW at row nu-2 inside ActAero==1, giving nu = 5 (ATD off) / 9 (ATD
%   on). Warm-start callers must pass getfielddef(vp,'rwMandate',0) - omit it and
%   latestInit's r==nu filter silently rejects every mode 6/7 cache, so the run
%   cold-starts instead of failing visibly.

if nargin < 3, rwMandate = 0; end
validateattributes(ATD,       {'numeric','logical'}, {'scalar','integer','>=',0,'<=',1}, mfilename, 'ATD');
validateattributes(ActAero,   {'numeric'},           {'scalar','integer','>=',0,'<=',3}, mfilename, 'ActAero');
validateattributes(rwMandate, {'numeric'},           {'scalar','integer','>=',0},        mfilename, 'rwMandate');

base = [3 4 5 7];              % ATD off, ActAero 0..3
if ATD == 1
    base = [7 8 9 11];         % ATD on
end
nu = base(ActAero + 1);
if ActAero == 1 && any(rwMandate == [6 7])
    nu = nu + 1;               % + activeAeroFW at row nu-2
end
end

function nu = nuForConfig(ATD, ActAero, rwMandate)
%NUFORCONFIG Number of control inputs for a (pt.ATD, vp.ActAero) combination.
%
%   nu = nuForConfig(pt.ATD, vp.ActAero)
%   nu = nuForConfig(pt.ATD, vp.ActAero, vp.rwMandate)
%
%   Mirrors the nu ladder in Scripts/vehModel.m, which is authoritative. This
%   exists because the warm-start lookup in the MLTP*.m entry points happens
%   BEFORE vehModel.m runs, so nu is not yet defined there - and without it
%   latestInit cannot tell an AWD cache (nu=4) from an ATD one (nu=8) for the
%   same aeroSetting, which is exactly how an ATD ActiveRW run ends up
%   warm-starting from an AWD lap.
%
%   Keep in step with vehModel.m's ladder. Per CLAUDE.md's config-matrix rule,
%   any change to the input set must be applied to EVERY branch of EVERY ladder -
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
%   rwMandate (optional, default 0 = mandate-independent legacy behaviour):
%   modes 6 (ARFWd) and 7 (ARFWr) are the mandates that change nu - both add
%   activeAeroFW at row nu-2 INSIDE ActAero==1, so nu = 5 (ATD off) / 9 (ATD
%   on) there. No other (ActAero, rwMandate) pairing changes the table.
%   Warm-start callers must pass getfielddef(vp,'rwMandate',0), or every
%   ARFWd/ARFWr cache is silently rejected by latestInit's r==nu filter and
%   the run cold-starts.

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
    nu = nu + 1;               % ARFWd/ARFWr: + activeAeroFW (row nu-2)
end
end

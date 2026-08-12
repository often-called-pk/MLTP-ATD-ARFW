function [clf, clr] = aeroEvalNum(c, vx)
%AEROEVALNUM Numeric evaluation of collapsed aero coefficient curves.
%   [clf, clr] = aeroEvalNum(coef, vx) with coef from aeroCollapse and vx in
%   m/s (scalar or vector). This is the numeric twin of aeroEvalSX at the
%   bottom of Scripts/vehModel.m - KEEP THE FORMULAS IDENTICAL, the constant-
%   coefficient init model (vehModel_initial.m) consumes these values.
qn = (vx.^2)/c.qs;
s1 = 0.5*(1 + tanh((vx - c.vc1)/c.wb));
s2 = 0.5*(1 + tanh((vx - c.vc2)/c.wb));
clf = (1-s1).*hornEval(c.pf1, qn) + s1.*(1-s2).*hornEval(c.pf2, qn) + s1.*s2*c.pf3;
clr = (1-s1).*hornEval(c.pr1, qn) + s1.*(1-s2).*hornEval(c.pr2, qn) + s1.*s2*c.pr3;
end

function y = hornEval(p, x)
% Horner evaluation, highest-order coefficient first (polyval-compatible)
y = zeros(size(x)) + p(1);
for k = 2:numel(p)
    y = y.*x + p(k);
end
end

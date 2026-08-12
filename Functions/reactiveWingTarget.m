function [tRW, tFW, sOn] = reactiveWingTarget(vx, r, Tbrake, TbMax, law)
%REACTIVEWINGTARGET Reactive dual-wing law: kappa_inst = r/vx classifier + brake overlay.
%  Twin-free (pure tanh/arith, SX==double) - same rule as rwVelocityTarget.
%  kap2 = r^2/(vx^2+vEps^2): smooth, sign-free, vEps=1 avoids 1/0 (bias <0.3% at vx>=14).
%  Blends compare kap2 to k1^2/k2^2 with widths 2*k1*w1 / 2*k2*w2 (w in 1/m).
%  Brake switch = rwVelocityTarget 3rd output (single owner; tv discarded).
if nargin < 5 || isempty(law), law = struct(); end
assert(isstruct(law) && isscalar(law), 'reactiveWingTarget:law', 'law must be a scalar struct');
k1   = getfielddef(law, 'k1',   0.004);   % straight|sweeper (1/m, R=250 m)
k2   = getfielddef(law, 'k2',   0.0167);  % sweeper|sharp    (1/m, R=60 m)
w1   = getfielddef(law, 'w1',   0.002);   % blend half-widths (1/m), capped <= (k2-k1)/4
w2   = getfielddef(law, 'w2',   0.003);
stRW = getfielddef(law, 'stRW', [-10 0 10 15]);   % [straight sweeper sharp brake]
stFW = getfielddef(law, 'stFW', [-25 -20 0]);     % [straight sweeper sharp]; brake=stFW(3)
bOpt = getfielddef(law, 'brakeOpts', struct('zeroTrim', true));
chk = @(v,n) assert(isnumeric(v) && isreal(v) && all(isfinite(v(:))), ...
    'reactiveWingTarget:optType', 'law.%s must be finite real numeric', n);
chk(k1,'k1'); chk(k2,'k2'); chk(w1,'w1'); chk(w2,'w2'); chk(stRW,'stRW'); chk(stFW,'stFW');
assert(k2 > k1 && k1 > 0 && w1 > 0 && w2 > 0, 'reactiveWingTarget:thresholds', ...
    'need 0 < k1 < k2 and positive widths');
assert(numel(stRW) == 4 && numel(stFW) == 3, 'reactiveWingTarget:stations', ...
    'stRW must have 4 entries, stFW 3');

assert(w1 <= (k2-k1)/4 + 1e-12 && w2 <= (k2-k1)/4 + 1e-12, ...
    'reactiveWingTarget:widthCap', 'need w1,w2 <= (k2-k1)/4 (plateau + zeroTrim validity)');

vEps = 1;                                   % m/s
kap2 = (r.*r) ./ (vx.*vx + vEps^2);
% zeroTrim renorm (brakeSwitch pattern): raw gate at kappa=0 is 0.5*(1+tanh(-k/(2w)))
% which does NOT saturate to 0 unless w << k - untrimmed, the straight station is
% never reached (same trap rwVelocityTarget.m:145-150 documents for sOn).
g1r = 0.5 .* (1 + tanh((kap2 - k1^2) ./ (2*k1*w1)));   % straight -> sweeper
g01 = 0.5 *  (1 + tanh(-k1/(2*w1)));                   % numeric constant, SX-safe
g1  = (g1r - g01) ./ (1 - g01);                        % g1(kappa=0) == 0 exactly
g2r = 0.5 .* (1 + tanh((kap2 - k2^2) ./ (2*k2*w2)));   % sweeper  -> sharp
g02 = 0.5 *  (1 + tanh(-k2/(2*w2)));
g2  = (g2r - g02) ./ (1 - g02);
tRWk = stRW(1) + (stRW(2)-stRW(1)).*g1 + (stRW(3)-stRW(2)).*g2;
tFWk = stFW(1) + (stFW(2)-stFW(1)).*g1 + (stFW(3)-stFW(2)).*g2;

[~, ~, sOn] = rwVelocityTarget(vx, Tbrake, TbMax, struct('brakeOpts', bOpt));

tRW = tRWk + sOn .* (stRW(4) - tRWk);
tFW = tFWk + sOn .* (stFW(3) - tFWk);
end

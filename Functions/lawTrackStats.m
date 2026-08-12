function st = lawTrackStats(alpha, target, band, band0, tolDeg)
%LAWTRACKSTATS Band-health / law-tracking scalars for ONE banded wing surface.
%
%   st = lawTrackStats(alpha, target, band, band0, tolDeg)
%
%   alpha/target/band are same-length knot traces (deg); band0 is the surface's
%   BASE band half-width (law.bandRW / law.bandFW), tolDeg the constraint
%   tolerance in degrees.
%
%   WHY THIS EXISTS. Mode 7's band opens wide near classifier transitions, so
%   occupancy alone saturates at 1.00 and says nothing: a satisfied constraint is
%   not evidence of a binding one (the ARWv lesson recorded at MLTP.m:977). The
%   numbers that DO say whether the law binds are the band width itself and the
%   tracking error restricted to the knots where the band is tight. Two call
%   sites need them - MLTP.m's mode-7 post-processing and the post-hoc recompute
%   for already-solved runs - so the arithmetic has ONE owner, same standing rule
%   as Functions/rwBasisWeights.m.
%
%   tolDeg MUST be IPOPT's own: opts.ipopt.constr_viol_tol * u_s(row) (~3e-3 deg
%   at 1e-4 / scale 30), the derivation MLTP.m:968/:1072 already use. A tighter
%   literal (the old 1e-6) counts knots IPOPT never promised to place.
%
%   FIELD MAP to mode 3's data.rwVel block (MLTP.m:973-989), so ARWv and ARFWr
%   numbers are read the same way:
%     st.errRMS      <- rwVel.errRMS         RMS |alpha-target| over all knots
%     st.errMaxAbs   <- rwVel.errMaxAbs      max |alpha-target|
%     st.bandMedian  <- rwVel.bandMedian     median band half-width (deg)
%     st.bandP90     <- rwVel.bandP90        90th pct band half-width (deg)
%     st.fracBand1   <- rwVel.fracBand1      frac of knots with band <= 1 deg
%     st.fracErr1    <- rwVel.fracErr1       frac of knots with |err| <= 1 deg
%     st.fracTight   <- rwVel.fracTight      frac with band <= band0*1.05
%     st.errRMStight <- rwVel.errRMStight    RMS |err| over those tight knots
%   st.occ has no mode-3 twin (mode 3's band is one-sided): frac of knots inside
%   the two-sided band, |err| <= band + tolDeg.
alpha  = reshape(full(alpha),  1, []);
target = reshape(full(target), 1, []);
band   = reshape(full(band),   1, []);
assert(numel(alpha) == numel(target) && numel(alpha) == numel(band), ...
    'lawTrackStats:size', 'alpha/target/band must be the same length (got %d/%d/%d)', ...
    numel(alpha), numel(target), numel(band));
assert(isscalar(band0) && isfinite(band0) && band0 > 0, ...
    'lawTrackStats:band0', 'band0 must be a positive finite scalar');
assert(isscalar(tolDeg) && isfinite(tolDeg) && tolDeg >= 0, ...
    'lawTrackStats:tol', 'tolDeg must be a non-negative finite scalar');

err = alpha - target;

st = struct();
st.errRMS     = sqrt(mean(err.^2));
st.errMaxAbs  = max(abs(err));
st.occ        = mean(abs(err) <= band + tolDeg);
st.bandMedian = median(band);
st.bandP90    = prctile(band, 90);
st.fracBand1  = mean(band <= 1.0);
st.fracErr1   = mean(abs(err) <= 1.0);

tight         = band <= band0*1.05;          % transition-free knots (mode-3 rule)
st.fracTight  = mean(tight);
st.nTight     = nnz(tight);
if any(tight)
    st.errRMStight = sqrt(mean(err(tight).^2));
else
    st.errRMStight = NaN;                    % no tight knot: band never closed
end
end

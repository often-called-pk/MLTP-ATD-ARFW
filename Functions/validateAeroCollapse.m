function results = validateAeroCollapse()
%VALIDATEAEROCOLLAPSE Aero-map integration validation (plan checklist 4.1-4.3).
%
%   1. Re-imports the workbook and spot-checks the 4 corner cells of each
%      CL table against independent single-cell reads.
%   2. Runs Parameters/vehParams.m (exercises the live integration path),
%      then collapses the map for all four digitised rear-wing angle NODES
%      (-10/0/10/15 deg -- unaffected by the 2026-08-04 run-config cull,
%      see rwAeroMap2D.m) using the force-equivalent RW-angle increments
%      (Functions/rwAeroDelta.m); aeroCollapse itself enforces the
%      convergence (<0.01 mm) and fit-residual (<=0.02) gates. The fifth
%      node, +20 deg, was dropped 2026-08-05 along with the ActiveRW map's
%      fifth node and the free wing's control bound (both capped at +15) -
%      see docs/superpowers/specs/2026-08-05-four-node-map-and-report-fixes-design.md
%      and Functions/rwAeroDelta.m's Provenance note; rwAeroDelta() now
%      errors on alpha=20, so this function can no longer collapse it.
%   3. Archives CL(v)/RH(v)/balance(v) + fit-overlay plots per node to
%      Validation/aero/ (gitignored - INTERNAL ONLY, confidential source).
%
%   Returns a struct with the per-node collapsed coefficient structs.
%   (The High->Shallow shed and its v_sw check were removed with the RW-angle
%   model: stall is now captured by the +15 deg data point itself. The +20 deg
%   point also fell in the separated regime before it was dropped as a node -
%   see the stall note in Functions/rwAeroDelta.m.)

thisDir  = fileparts(mfilename('fullpath'));
repoRoot = fileparts(thisDir);
outDir   = fullfile(repoRoot, 'Validation', 'aero');
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% 4.1 - import + corner-cell spot checks
map  = importAeroMap();
xlsx = fullfile(repoRoot, 'reference', 'aero', 'SportsCar_AeroBalance_ver1_0.xlsx');
if ~isfile(xlsx)      % pre-2026-08-03 layout: workbook still at the repo root
    xlsx = fullfile(repoRoot, 'SportsCar_AeroBalance_ver1_0.xlsx');
end
spot = @(rng) readmatrix(xlsx, 'Sheet', 'Lift_Downforce', 'Range', [rng ':' rng]);
checks = {  % table  cell   [row col] in the 10x11 table
    'CLf', 'C11', [ 1  1];  'CLf', 'M11', [ 1 11];
    'CLf', 'C20', [10  1];  'CLf', 'M20', [10 11];
    'CLr', 'C24', [ 1  1];  'CLr', 'M24', [ 1 11];
    'CLr', 'C33', [10  1];  'CLr', 'M33', [10 11]};
fprintf('--- corner-cell spot checks ---\n');
for i = 1:size(checks, 1)
    tbl = checks{i,1};  cellRef = checks{i,2};  rc = checks{i,3};
    vSheet = spot(cellRef);
    vMap   = map.(tbl)(rc(1), rc(2));
    fprintf('  %s(%2d,%2d) = %+8.4f  | sheet %s = %+8.4f\n', tbl, rc(1), rc(2), vMap, cellRef, vSheet);
    assert(abs(vSheet - vMap) < 1e-12, 'spot check failed: %s %s', tbl, cellRef);
end
fprintf('  all 8 corner cells match.\n');

%% 4.2 - live vehParams path, then collapse every aero node
run(fullfile(repoRoot, 'Parameters', 'vehParams.m'));     % defines vp (+ runs collapse for the selected setting)
% ANGLE-NODE names, not run configs: these are the four digitised wing-angle
% nodes (see rwAeroMap2D.m's alphaNodes), unaffected by the 2026-08-04
% run-config cull that dropped 'RWp20' from runConfigs(). This function feeds
% angles(i) to rwAeroDelta() directly below (never reloads vehParams per
% "setting"), so there is no vp.aeroSetting == 'RWp20' path here to break --
% nodeNames is purely a labelling/struct-field convenience. The fifth node,
% 'RWp20'/20 deg, was dropped 2026-08-05 (see the header note above); keeping
% it here would now hard-error, since rwAeroDelta() no longer accepts alpha=20.
nodeNames = {'Low', 'Mid', 'High', 'RWp15'};
angles    = [ -10,    0,     10,     15   ];      % rear-wing angle per node (deg)
results  = struct();
fprintf('--- collapse per aero node (gates enforced inside aeroCollapse) ---\n');
for i = 1:numel(nodeNames)
    s = nodeNames{i};
    [dClfA, dClrA, ~] = rwAeroDelta(angles(i));           % force-equivalent products
    results.(s) = aeroCollapse(map, vp.RHf0, vp.RHr0, 2*vp.kwf, 2*vp.kwr, ...
                  dClfA/vp.A, dClrA/vp.A, vp.rho, vp.A, 125);
end

% 4.3 - the FRONT clamp onset should sit inside the raced speed range
% (~66-78 m/s with the placeholder RH0); the rear onset varies legitimately
% per node (Low = -10 deg gives the least rear downforce and may never
% clamp). Warn, don't fail - placeholders may move these.
for i = 1:numel(nodeNames)
    s   = nodeNames{i};
    vcF = results.(s).raw.vcF_raw;
    if isnan(vcF) || vcF < 55 || vcF > 95
        warning('validateAeroCollapse:onsetBand', ...
            '%s: front clamp onset %s m/s outside the expected 55-95 band', s, num2str(vcF));
    end
end

%% 4.2(c) - archive plots (INTERNAL ONLY - Validation/ is gitignored)
for i = 1:numel(nodeNames)
    s   = nodeNames{i};
    c   = results.(s);
    v   = c.raw.v;
    [clfFit, clrFit] = aeroEvalNum(c, v);

    fig = figure('Visible', 'off', 'Position', [50 50 1200 800]);
    subplot(2,2,1); hold on; grid on;
    plot(v, c.raw.clf, 'b-', v, clfFit, 'b--', v, c.raw.clr, 'r-', v, clrFit, 'r--');
    if c.vc1 <= v(end), xline(c.vc1, 'k:'); end
    if c.vc2 <= v(end), xline(c.vc2, 'k:'); end
    legend('CL_f collapse','CL_f fit','CL_r collapse','CL_r fit','Location','best');
    xlabel('v [m/s]'); ylabel('CL [-]'); title(sprintf('%s: collapsed vs fitted', s));

    subplot(2,2,2); hold on; grid on;
    plot(v, clfFit - c.raw.clf, 'b', v, clrFit - c.raw.clr, 'r');
    yline(0.02, 'k--'); yline(-0.02, 'k--');
    legend('front','rear','Location','best');
    xlabel('v [m/s]'); ylabel('fit - collapse [-]');
    title(sprintf('residuals (max F %.4f / R %.4f)', c.residF, c.residR));

    subplot(2,2,3); hold on; grid on;
    plot(v, c.raw.RHf, 'b', v, c.raw.RHr, 'r');
    yline(min(map.RHf), 'b:'); yline(min(map.RHr), 'r:');
    legend('RH_f','RH_r','Location','best');
    xlabel('v [m/s]'); ylabel('ride height [mm]');
    title(sprintf('platform (clamp onset F %.0f / R %.0f m/s)', c.raw.vcF_raw, c.raw.vcR_raw));

    subplot(2,2,4); hold on; grid on;
    plot(v(2:end), 100*c.raw.bal(2:end), 'k');
    xlabel('v [m/s]'); ylabel('front balance [%]');
    title('aero balance CL_f/(CL_f+CL_r)');

    [dClfA_i, dClrA_i, ~] = rwAeroDelta(angles(i));
    sgtitle(sprintf('aeroCollapse - %s (RW %+d deg: dClF %+.2f, dClR %+.2f) - CONFIDENTIAL, internal only', ...
        s, angles(i), dClfA_i/vp.A, dClrA_i/vp.A));
    pBase = fullfile(outDir, sprintf('aeroCollapse_%s', s));
    print(fig, [pBase '.png'], '-dpng', '-r300');
    try, savefig(fig, [pBase '.fig']); catch, end
    close(fig);
end

save(fullfile(outDir, 'aeroCollapse_validation.mat'), '-struct', 'results');
fprintf('validateAeroCollapse: PASS - %d RW-angle nodes collapsed; artifacts in %s\n', ...
    numel(nodeNames), outDir);
end

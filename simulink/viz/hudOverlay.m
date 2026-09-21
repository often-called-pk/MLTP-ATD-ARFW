function img = hudOverlay(img, S)
%HUDOVERLAY  Burn a driver-instrument HUD into one rendered RGB frame.
%
%   img = HUDOVERLAY(img, S)
%
%   img  m-by-n-by-3 uint8 RGB, exactly as sim3d.sensors.IdealCamera/read
%        returns it. The class and the SIZE are preserved -- unrealPlayback
%        hands the result straight to writeVideo, and wingStateMontage
%        asserts that every still it stacks shares one size.
%
%   S    one frame's telemetry, all scalars unless noted:
%          vx        [m/s]  road speed          (VehStateBus.vx)
%          steerDeg  [deg]  FRONT ROAD-WHEEL angle, + = LEFT (DriverCmdBus.steer,
%                           logged in rad -- convert before calling). This is
%                           NOT a hand-wheel angle; peak on a solved lap is
%                           ~11 deg, which is why it is labelled in the frame.
%          gear      [-]    indicated gear 1..8 from indicatedGear(), or NaN
%                           to draw '--'. DISPLAY ONLY -- see that function's
%                           header; the plant runs one fixed ratio and never
%                           shifts.
%          throttle  [0..1] DriverCmdBus.accel, POST traction ceiling
%          brake     [0..1] DriverCmdBus.brake, POST traction ceiling
%          aRW, aFW  [deg]  logged actuator positions (already degrees)
%          colRW,colFW [1x3] 0..1 RGB station colours (unrealPlayback's palette)
%          tLap      [s]    lap clock, or NaN to omit the chip
%          note      char    optional one-line caveat drawn top-right (used for
%                           the cosmetic wheel-spin scaling); '' = omit
%
%   REMOVED 2026-09-17: the optional 'credit' field, a faint one-line asset
%   attribution drawn above the bar's left edge. It existed for an imported
%   third-party body whose licence asked for on-screen attribution; that body
%   and its caller-side option are gone, the one car now carries its licence
%   and provenance in its own LICENSE.md beside the mesh, and a field nothing
%   supplies is a trap for the next reader rather than a feature.
%
%   Requires Computer Vision Toolbox (insertText / insertShape). The caller
%   gates the licence once before the render and switches the HUD off with a
%   warning if it is unavailable -- never per frame.
%
%   Geometry is authored in a 1280x720 reference frame and scaled by the
%   actual image width, so a different 'ImageSize' keeps its proportions.
%
%   See also unrealPlayback, indicatedGear.

W  = size(img, 2);
H  = size(img, 1);
sc = W/1280;

barH = 128;
barY = 720 - barH;          % reference-space top of the bar

% ---- shape accumulators -------------------------------------------------
tRect = zeros(0,4); tCol = zeros(0,3);      % translucent
oRect = zeros(0,4); oCol = zeros(0,3);      % opaque
% ---- text accumulator ---------------------------------------------------
tx = struct('pos', {}, 'str', {}, 'fs', {}, 'col', {}, 'anc', {});

    function addT(x, y, str, fs, col, anc)
        if nargin < 6, anc = 'LeftTop'; end
        tx(end+1) = struct('pos', [x y], 'str', str, 'fs', fs, 'col', col, 'anc', anc);
    end

INK   = [244 246 250];
DIM   = [152 160 176];
FAINT = [126 132 146];
PANEL = [ 16  18  24];

% ===== panel + top-left lap chip (translucent) ===========================
tRect(end+1,:) = [0 barY 1280 barH];  tCol(end+1,:) = PANEL;
if isfield(S,'tLap') && ~isempty(S.tLap) && ~isnan(S.tLap)
    tRect(end+1,:) = [20 16 210 36];  tCol(end+1,:) = PANEL;
    addT(34, 23, sprintf('LAP  %6.2f s', S.tLap), 19, INK);
end

% top accent line + column separators
oRect(end+1,:) = [0 barY 1280 2];  oCol(end+1,:) = [232 236 242];
for xs = [262 482 732 1022]
    oRect(end+1,:) = [xs barY+16 1 barH-32];  oCol(end+1,:) = [86 92 104]; %#ok<AGROW>
end

% ===== 1. speed ==========================================================
addT( 24, barY+12, 'SPEED', 13, DIM);
addT( 24, barY+26, sprintf('%3.0f', S.vx*3.6), 54, INK);   % signed, same as the m/s line
addT(178, barY+58, 'km/h', 18, DIM);
addT( 24, barY+100, sprintf('%.1f m/s', S.vx), 13, FAINT);

% ===== 2. gear (display only) ============================================
addT(274, barY+12, 'GEAR (indicated)', 13, DIM);
if isfield(S,'gear') && ~isempty(S.gear) && ~isnan(S.gear)
    gStr = sprintf('%d', round(S.gear));
else
    gStr = '--';
end
addT(276, barY+26, gStr, 54, INK);
addT(274, barY+ 90, 'derived from wheel speed', 11, FAINT);
addT(274, barY+105, 'plant runs one ratio, never shifts', 11, FAINT);

% ===== 3. steer ==========================================================
addT(494, barY+12, 'STEER (road-wheel, + = left)', 13, DIM);
addT(494, barY+32, sprintf('%+.1f deg', S.steerDeg), 30, INK);
sx = 494; sw = 226; sy = barY+86;
oRect(end+1,:) = [sx sy sw 12];  oCol(end+1,:) = [56 60 70];
cxs = sx + sw/2;
sFrac = max(-1, min(1, S.steerDeg/12));
if abs(sFrac) > 1e-3
    % + = LEFT, so a positive value fills LEFTWARD from the centre tick and
    % a negative one rightward -- the bar must agree with its own label.
    wBar = abs(sFrac)*sw/2;
    x0   = cxs - max(0, sFrac)*sw/2;
    oRect(end+1,:) = [x0 sy wBar 12];  oCol(end+1,:) = [ 82 186 255];
end
oRect(end+1,:) = [cxs-1 sy-5 2 22];  oCol(end+1,:) = [206 210 220];

% ===== 4. throttle / brake ==============================================
addT(744, barY+12, 'THROTTLE / BRAKE', 13, DIM);
pedal = { 'THR', max(0,min(1,S.throttle)), [ 74 200 104], barY+34; ...
          'BRK', max(0,min(1,S.brake)),    [232  78  64], barY+72};
for i = 1:2
    yb = pedal{i,4};
    addT(744, yb+2, pedal{i,1}, 14, DIM);
    oRect(end+1,:) = [792 yb 176 20];              oCol(end+1,:) = [56 60 70]; %#ok<AGROW>
    if pedal{i,2} > 1e-3
        oRect(end+1,:) = [792 yb 176*pedal{i,2} 20]; oCol(end+1,:) = pedal{i,3}; %#ok<AGROW>
    end
    addT(1010, yb+1, sprintf('%3.0f%%', 100*pedal{i,2}), 17, INK, 'RightTop');
end

% ===== 5. wings ==========================================================
addT(1034, barY+12, 'ACTIVE AERO', 13, DIM);
wing = {'RW', S.aRW, S.colRW, barY+32; 'FW', S.aFW, S.colFW, barY+66};
for i = 1:2
    yb = wing{i,4};
    oRect(end+1,:) = [1034 yb 16 22];  oCol(end+1,:) = round(255*wing{i,3}(:).'); %#ok<AGROW>
    addT(1060, yb+1, sprintf('%s %+5.1f deg', wing{i,1}, wing{i,2}), 18, INK);
end
addT(1034, barY+100, 'chip = station in force', 11, FAINT);

% ===== optional caveat line, top right ===================================
if isfield(S,'note') && ~isempty(S.note)
    addT(1260, 23, S.note, 13, [214 200 140], 'RightTop');
end

% ===== draw ==============================================================
if ~isempty(tRect)
    img = insertShape(img, 'filled-rectangle', localScale(tRect, sc, W, H), ...
        ShapeColor = uint8(tCol), Opacity = 0.62);
end
if ~isempty(oRect)
    img = insertShape(img, 'filled-rectangle', localScale(oRect, sc, W, H), ...
        ShapeColor = uint8(oCol), Opacity = 1);
end

fsAll  = round(max(8, [tx.fs]*sc));
ancAll = {tx.anc};
key    = strcat(cellstr(num2str(fsAll(:))), '|', ancAll(:));
[uk, ~, ig] = unique(key);
for i = 1:numel(uk)
    sel = (ig == i);
    pos = vertcat(tx(sel).pos);
    % the bar (reference y >= barY) is pinned to the image floor; the top
    % chips keep their distance from the image top under any 'ImageSize'
    yOff = (pos(:,2) >= barY) * (H - 720*sc);
    pos  = [pos(:,1)*sc, pos(:,2)*sc + yOff];
    img = insertText(img, pos, {tx(sel).str}, ...
        FontSize = fsAll(find(sel,1)), TextColor = uint8(vertcat(tx(sel).col)), ...
        BoxOpacity = 0, AnchorPoint = ancAll{find(sel,1)});
end
end

% =========================================================================
function R = localScale(R, sc, ~, H)
%LOCALSCALE Reference-frame [x y w h] -> pixel rectangles. Rectangles that
% belong to the bottom bar (reference y >= 720-128) are pinned to the image
% floor so a non-16:9 'ImageSize' still puts the bar on the floor; the
% top-left chip keeps its distance from the image top.
inBar      = R(:,2) >= 720 - 128;
R(:,[1 3]) = R(:,[1 3])*sc;
R(:,[2 4]) = R(:,[2 4])*sc;
R(inBar,2) = R(inBar,2) + (H - 720*sc);
end

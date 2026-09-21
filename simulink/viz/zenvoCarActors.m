function varargout = zenvoCarActors(world, vp, M)
%ZENVOCARACTORS  The one car: a Zenvo Aurora Tur body with the Agil's rear wing
% transplanted onto its deck, and its own front-bumper vent band turned into a
% real moving flap, under one ISO8855 root.
%
%   H = ZENVOCARACTORS(world, vp, M)
%   [deployDeg, tuckDeg, fcn] = ZENVOCARACTORS('hingeMap')
%
%   M is the car manifest, resolved through assetManifest('Zenvo'), and is the
%   ONLY source of asset geometry: no node name, box, hub, pivot or scale factor
%   is written down in this file.
%
% WHAT IT BUILDS
%
%   Root ---- Body ------ <980 loaded nodes>   originals; the wheels and the
%   |                                          nose leaves below are hidden
%   |    ---- Skin01..0N                       those nose leaves REBUILT with the
%   |                                          two flank duct strips cut out of
%   |                                          them (the centre vent is left
%   |                                          untouched, still on Body)
%   +------- SteerFL/FR -- SpinFL/FR -- WheelFL/FR   rotor copies (spin)
%   |            +-------- HubFL/FR                  stator copies (steer only)
%   +------- SpinRL/RR ---- WheelRL/RR
%   +------- HubRL/RR
%   +------- WingFWHinge -- WingFW -+- WingFWBandR    the Tur's own flank duct
%   |                               +- WingFWBandL    strips, one hinge, one band
%   |                                                 ribbon per strip
%   +------- WingRW ------- WingRWPlane - WingRWBand  the Agil's wing plane
%   +------- WingRWPylons                             the Agil's struts (static)
%   +------- CamRig
%
%   ACTOR NAMES ARE FROZEN AND UNPREFIXED, because the live Simulink route
%   resolves actors BY NAME:
%
%     Root Body CamRig
%     SteerFL SteerFR                 front uprights (yaw = steer)
%     SpinFL SpinFR SpinRL SpinRR     rotating hubs (pitch = wheel spin)
%     WheelFL WheelFR WheelRL WheelRR the rotor copies (rim + disc + tyre)
%     HubFL HubFR HubRL HubRR         the stator copies (calliper)
%     WingRW WingRWPylons WingRWBand  rear wing pivot / struts / station band
%     WingFWHinge WingFW             front flap hinge / merged two-strip plate
%     WingFWBandR WingFWBandL        one station-band ribbon per strip
%
%   The caller drives exactly two aero handles:
%     WingRW.Rotation      = [0 aRW 0]           + = trailing edge up
%     WingFWHinge.Rotation = [0 fcn(aFW) 0]      fcn from the static query form
%   The delivered geometry is station 0 for the wing and the TUCKED (flush) state
%   for the flap, so both read correctly with no command at all. Both strips
%   share the one hinge, so they always move together.
%
%   The two station palettes are handed back as H.palRW (4x3) and H.palFW (3x3),
%   so a caller writing WingRWBand.Color / WingFWBand(i).Color per station
%   consumes them from here instead of keeping its own copy of dashboardApp's
%   chips. H.fwBand is a 1x2 actor array [WingFWBandR WingFWBandL] (H.fwBands is
%   the same array under the plural name, for callers that prefer it).
%
%   Two kinds of actor are created outside that list. WingRWPlane exists because
%   the two rotations do not commute: the extracted mesh has to be yawed onto ISO
%   by M.donor.upAxisFix FIRST and pitched about the pivot SECOND, so the pitch
%   actor (WingRW, the name the route writes) must be the PARENT of the actor that
%   carries the mesh. WingFW is the same thing on the front, and there the plan
%   already names both halves. And any extracted part made of more than one
%   MATERIAL gets one extra child actor per extra material (WingFWM2, ...) --
%   see note 3.
%
% THE FOUR THINGS THIS FILE HAS TO GET RIGHT
%
% 1. THE FRAME, AND THE ENGINE CONVENTION THAT BITES. Everything read out of a
%    loaded leaf is in the ASSET frame (the leaf's own .Vertices plus the
%    accumulated Translations of the nodes above it): x lateral, y longitudinal
%    with the NOSE at +y, z up, tyres already on z = 0. Measured 2026-09-17 by
%    render (simulink/work/zm1_top.png, READ), on an ISO8855 actor:
%
%      createMesh(a, V, N, F, ...)  RENDERS AT V. Pass the asset vertices
%                                   STRAIGHT THROUGH, original winding.
%      a.Vertices                   then hands back [V(:,1) -V(:,2) V(:,3)],
%      a.Faces                      and F(:,[1 3 2]).
%
%    The flip lives in the GETTER, not in the renderer. localMeshWrite below
%    therefore writes straight through and checks the READ-BACK against the
%    negated vertices and the swapped winding. The deleted carAssetActors
%    negated before writing and compared against the un-negated vertices: that
%    round trip passes while drawing every copy mirrored end-for-end, which on
%    these y-long bodies puts the nose at the tail. The decisive frame drew the
%    same cube both ways AND the Tur's own front half both ways; only the
%    straight-through copies landed at the nose.
%
%    The consequence for placement, also measured on that frame: an asset point p
%    drawn on an actor carrying upAxisFix as its Rotation renders at the ISO point
%    [p(2) p(1) p(3)] plus the actor's translation. That is localIso below. It is
%    NOT a rotation (its determinant is -1) precisely because the engine's vertex
%    y flip is folded into it, and it is why the manifest can say "ISO y is the
%    asset's +x" while the yaw is +90.
%
%    A CHILD actor of one of these, with no rotation and no translation of its
%    own, inherits exactly that map -- so its mesh can be written in the SAME
%    asset coordinates as its parent's and lands on it. Every station band and
%    every extra-material part below relies on that, and it is render-verified.
%
% 2. STATION. The mesh is drawn at its own scale (M.body.scale) and the Body is
%    translated in x so the mesh's OWN measured axle midpoint lands at
%    (vp.l_f - vp.l_r)/2, which is where vp's reference sits relative to the
%    wheelbase midpoint. The hubs that fall out of that are then ASSERTED against
%    vp.l_f / -vp.l_r at 1 cm: the scale is what has to make them agree, so a
%    mismatch is a manifest error, not something to nudge here.
%
% 3. REAL PARTS MOVE, AND THEY KEEP THEIR SKIN. Neither the flap nor the wing is
%    a box plate. The flap is the Tur's own lower-bumper vent band, split face by
%    face out of the three shells that carry it; the wing and its struts are split
%    out of the Agil, which is then removed from the world entirely. The split is
%    on each face's CENTROID, which makes it a genuine PARTITION: the part and the
%    hole it leaves are complementary, never doubled (the plan forbids that
%    outright) and never separated by a gap.
%
%    Everything this asset draws is a TEXTURE on a white material -- measured, all
%    four extracted parts come back Color [1 1 1] with Metallic 0.70 and a .png --
%    so an extraction that dropped the texture renders a white wing, white tyres
%    and a white flap, which is exactly what the first pass did
%    (simulink/work/zb1_wheel.png). Each extracted part is therefore grouped by
%    MATERIAL and rebuilt as one actor per material, carrying that material's
%    texture and its texture coordinates; the largest group takes the frozen name
%    and the rest hang off it as children with no transform of their own (note 1).
%    Measured groups: 1 for each wheel rotor and each calliper, 2 for the flap
%    (both strips combined -- WingFW + WingFWM2), 2 for the wing and 2 for the
%    struts.
%
%    If any extraction comes back under MIN_TRIS the box is culled anyway and a
%    plain plate of the box's size is fitted in its place, with a warning.
%
%    ONE CAVEAT worth reading before trusting H.stats.feetZ: the pylon "feet" are
%    the manifest's CUT LINE, not the lowest vertex the struts draw. The drawn
%    mesh reaches about 21 mm below it, so the struts are planted THROUGH the
%    Tur's deck skin rather than resting on top of it -- deliberate, because a
%    buried foot cannot be seen and a floating one can. H.stats.feetZMesh is the
%    other number.
%
% 4. ROTOR vs STATOR, WITHOUT PER-CORNER NODES. This asset puts all four corners
%    under ONE instanced group, so the corner split is GEOMETRIC (a leaf belongs
%    to the corner whose manifest hub its bbox centre is within M.wheelGrabR of)
%    and the rotor/stator split is by ANCESTOR PATH (M.wheelRotorPat /
%    M.wheelStatorPat). Rotors go on Wheel<w> under Spin<w> and spin; stators go
%    on Hub<w> under Steer<w> (front) or Root (rear) and never spin.
%
%   See also assetManifest, test_carAsset, unrealPlayback.

% ---- static query form: no world, no vp, no actors, no engine -----------
if nargin >= 1 && (ischar(world) || isstring(world))
    if ~strcmpi(char(world), 'hingeMap')
        error('zenvoCarActors:query', ...
            ['zenvoCarActors: unknown static query ''%s''. The only one is ' ...
             '''hingeMap''; the build form takes a sim3d.World first.'], char(world));
    end
    [deployDeg, tuckDeg, fcn] = localHingeMap();
    varargout = {deployDeg, tuckDeg, fcn};
    return
end

narginchk(3, 3);
MIN_TRIS   = 100;      % plan spec 5: below this, cull and fit a plain plate
% The two band constants below are in RENDERED metres and are NOT scaled by the
% asset scale: they are this file's own styling, not asset-native lengths the way
% M.band and the manifest's boxes are. Scaling them would shrink the drawn band
% with the mesh and mean a different thing at every scale.
BAND_INSET = 0.02;     % m, total lateral inset of a station band
BAND_T     = 0.016;    % m, station-band thickness across its own edge
NBIN       = 28;       % spanwise stations a station band follows its edge through
% The FRONT flap's two bands get their own, much lower count. A station is
% dropped when fewer than 3 vertices fall within half a station of it, and the
% ribbon then bridges the gap with one stretched quad -- which is what broke the
% deployed flap into disconnected slivers. Measured on the shipped strip
% (simulink/work/t1bf_nbin.m, 330 faces / 356 vertices over a 0.40 m span):
% 28 stations -> 9 of 29 dropped, worst bridge 57.8 mm, ridden edge moves
% 36.6 mm; 12 or fewer -> NO station dropped at all. 10 is taken (11 stations,
% 40.5 mm apart, edge movement 22.6 mm) -- the wing's own band keeps NBIN,
% because it rides a 2 m span, not a 0.4 m one.
NBIN_FW    = 10;       % ... and the front flap's two, over its much shorter strip

if isempty(vp)
    error('zenvoCarActors:noVp', ...
        ['zenvoCarActors: a vehicle-parameter struct is required (vp.l, vp.l_f, ' ...
         'vp.l_r, vp.Rw, vp.t). Open simulink/ARFWr_RT.prj, which loads vp.']);
end
req = {'l','l_f','l_r','Rw','t'};
missing = req(~isfield(vp, req));
if ~isempty(missing)
    error('zenvoCarActors:vpFields', 'zenvoCarActors: vp is missing {%s}.', ...
        strjoin(missing, ', '));
end
reqM = {'body','donor','wheelHub','wheelGrabR','wheelRotorPat','wheelStatorPat', ...
        'flapBoxes','flapHinge','wingBox','pylonBox','wingPivot','wingPlace', ...
        'deckZ','pylonFeetZ','band','hideNodes'};
missing = reqM(~isfield(M, reqM));
if ~isempty(missing)
    error('zenvoCarActors:manifest', ...
        ['zenvoCarActors: the manifest is missing {%s}. Manifests are resolved ' ...
         'through assetManifest(name).'], strjoin(missing, ', '));
end
% SCALAR scales only: every point below is multiplied by one of these, so a 1x3
% would broadcast element-wise and land the car somewhere plausible and wrong.
for f = {'body','donor'}
    s = M.(f{1}).scale;
    if ~isnumeric(s) || ~isscalar(s) || ~isfinite(s) || s <= 0
        error('zenvoCarActors:scale', ...
            'zenvoCarActors: M.%s.scale must be a positive finite scalar (got %s).', ...
            f{1}, mat2str(s));
    end
end
sc  = M.body.scale;
scD = M.donor.scale;

% dashboardApp / unrealPlayback station palettes. The bands start on the
% max-downforce chip of each, so an un-driven rig still reads as a station.
palRW = [0.12 0.47 0.71; 0.50 0.50 0.50; 0.84 0.15 0.16; 1.00 0.60 0.00];
palFW = [0.20 0.63 0.79; 0.55 0.55 0.55; 0.55 0.10 0.60];

corners = {'FL','FR','RL','RR'};
rt = 0;                                   % worst mesh read-back residual so far

H = struct();
H.root = localIsoActor(world, 'Root', []);

% =========================================================================
% 1. THE BODY
% =========================================================================
H.body = localIsoActor(world, 'Body', H.root);
load(H.body, M.body.file, sc);
H.body.Rotation = M.body.upAxisFix;                 % deg, ISO8855
% The mesh's own axle midpoint, on its own long axis, onto vp's reference.
axleMid = (M.wheelHub.FL(2) + M.wheelHub.RL(2))/2 * sc;
dxCG    = (vp.l_f - vp.l_r)/2;
Tb      = [dxCG - axleMid, 0, 0];
H.body.Translation = Tb;

L = localWalk(H.body, [0 0 0], '');                 % leaf records, asset frame
if isempty(L)
    error('zenvoCarActors:emptyBody', ...
        'zenvoCarActors: %s loaded no geometry.', M.body.file);
end
bbAll = localUnionBox(L);

% Manifest nodes to suppress outright (none on this asset today).
for i = 1:numel(M.hideNodes)
    for j = 1:numel(L)
        if strcmp(L{j}.name, M.hideNodes{i}), L{j}.actor.Hidden = true; end
    end
end

% =========================================================================
% 2. THE WHEELS
% =========================================================================
H.steer = struct(); H.spin = struct(); H.wheel = struct(); H.hub = struct();
H.hubISO = struct();
wheelR = zeros(1,4);  wheelOff = 0;  nWheelLeaf = 0;
isRot = false(numel(L),1);  isSta = isRot;
for j = 1:numel(L)
    isRot(j) = contains(L{j}.path, M.wheelRotorPat);
    isSta(j) = contains(L{j}.path, M.wheelStatorPat);
end
for i = 1:numel(corners)
    k    = corners{i};
    hA   = M.wheelHub.(k) * sc;                     % hub, asset frame
    hISO = localIso(hA) + Tb;                       % hub, ISO, under the root
    want = vp.l_f;
    if k(1) ~= 'F', want = -vp.l_r; end
    if abs(hISO(1) - want) > 0.01
        error('zenvoCarActors:hubs', ...
            ['zenvoCarActors: corner %s lands at x = %.4f m but vp puts that axle ' ...
             'at %.4f m (%.1f mm out). M.body.scale (%g) is what has to make the ' ...
             'mesh''s own wheelbase agree with vp.l = %.3f m -- re-measure the ' ...
             'asset rather than nudging the hub.'], ...
            k, hISO(1), want, 1000*abs(hISO(1) - want), sc, vp.l);
    end
    H.hubISO.(k) = hISO;

    if k(1) == 'F'
        H.steer.(k) = localIsoActor(world, ['Steer' k], H.root);
        H.steer.(k).Translation = hISO;
        H.spin.(k)  = localIsoActor(world, ['Spin' k], H.steer.(k));
        hubParent = H.steer.(k);  hubT = [0 0 0];
    else
        H.spin.(k)  = localIsoActor(world, ['Spin' k], H.root);
        H.spin.(k).Translation = hISO;
        hubParent = H.root;  hubT = hISO;
    end

    near = false(numel(L),1);
    for j = 1:numel(L)
        c = (L{j}.bb(1,:) + L{j}.bb(2,:))/2;
        near(j) = norm(c - hA) <= M.wheelGrabR * sc;
    end
    Pw = localLeafPieces(L, near & isRot);
    Ph = localLeafPieces(L, near & isSta);
    if isempty(Pw) || isempty(Ph)
        error('zenvoCarActors:noWheelLeaf', ...
            ['zenvoCarActors: corner %s matched %d rotor and %d calliper leaves. ' ...
             'M.wheelGrabR (%g m) and the rotor/stator ancestor patterns are what ' ...
             'select them.'], k, numel(Pw), numel(Ph), M.wheelGrabR*sc);
    end
    [H.wheel.(k), ~, r1, Vw] = localBuildParts(world, H.spin.(k), ['Wheel' k], ...
                                               Pw, hA, M.body.upAxisFix);
    [H.hub.(k),   ~, r2]     = localBuildParts(world, hubParent,  ['Hub' k], ...
                                               Ph, hA, M.body.upAxisFix);
    H.hub.(k).Translation = hubT;
    rt = max([rt r1 r2]);
    nWheelLeaf = nWheelLeaf + nnz(near & (isRot | isSta));

    % A rotor re-centred on its own hub must come out centred on the origin --
    % the one cheap check that catches a corner paired with the wrong hub.
    cw = (max(Vw,[],1) + min(Vw,[],1))/2;
    if max(abs(cw)) > 0.02
        error('zenvoCarActors:hubPairing', ...
            ['zenvoCarActors: corner %s re-centres to %s instead of the origin. ' ...
             'M.wheelHub.%s and M.wheelGrabR are what pair the leaves with the ' ...
             'hub.'], k, mat2str(round(cw,4)), k);
    end
    wheelR(i) = (max(Vw(:,3)) - min(Vw(:,3)))/2;
    wheelOff  = max(wheelOff, max(abs(cw)));
end
% Hide the originals only now: the merge above reads them.
for j = 1:numel(L)
    if isRot(j) || isSta(j), L{j}.actor.Hidden = true; end
end

% =========================================================================
% 3. THE FLAP: the Tur's own two flank duct strips, cut out of the nose and
%    merged onto ONE hinge (the centre vent is left in the body, untouched)
% =========================================================================
% M.flapBoxes is a 2x6 LIST -- row 1 the +x (right) strip, row 2 the -x (left)
% strip, each its own box. Extracted in ONE localExtractMulti pass (its own
% MIN_TRIS fallback below is PER STRIP, not on the combined total): several of
% the Tur's own nose shells are wide enough to touch BOTH boxes at once (the
% "Base Coloured" body panel and the carbon lower fascia both span most of the
% car's width), so a leaf touched by both boxes must be rebuilt ONCE with both
% strips' faces removed together -- two independent single-box passes would
% each rebuild that same leaf missing only its OWN strip's hole, doubling it.
fbR    = M.flapBoxes(1,:) * sc;
fbL    = M.flapBoxes(2,:) * sc;
hingeA = mean(M.flapHinge, 1) * sc;               % the hinge MIDPOINT, both rows
[Pfs, nCullNose, nSkin, rtS] = localExtractMulti(world, H.root, L, {fbR, fbL}, ...
                                                 M.body.upAxisFix, Tb, 'Skin');
PfR = Pfs{1};  PfL = Pfs{2};
rt = max(rt, rtS);
trisR = localTotalPieceTris(PfR);
trisL = localTotalPieceTris(PfL);
% Fallback is PER STRIP: localPlatePieces returns a mirrored PAIR (it is built
% for a symmetric caller), so only piece 1 -- the one built at the given box's
% own centre -- is kept; the mirror half is not this strip's fallback.
if trisR < MIN_TRIS
    warning('zenvoCarActors:flapFallbackR', ...
        ['zenvoCarActors: the RIGHT flap-strip box came out with only %d ' ...
         'triangles, so it is a PLAIN PLATE of the box''s size instead of the ' ...
         'asset''s own geometry. Re-measure M.flapBoxes(1,:).'], trisR);
    Pp = localPlatePieces(fbR);  PfR = Pp(1);  trisR = localTotalPieceTris(PfR);
end
if trisL < MIN_TRIS
    warning('zenvoCarActors:flapFallbackL', ...
        ['zenvoCarActors: the LEFT flap-strip box came out with only %d ' ...
         'triangles, so it is a PLAIN PLATE of the box''s size instead of the ' ...
         'asset''s own geometry. Re-measure M.flapBoxes(2,:).'], trisL);
    Pp = localPlatePieces(fbL);  PfL = Pp(1);  trisL = localTotalPieceTris(PfL);
end
Pf = [PfR, PfL];

% The merged mesh: ONE hinge, ONE plate (still grouped by material -- note 3),
% re-centred on the hinge line. minTris=1 so localSurface's OWN fallback never
% fires here -- both strips were already validated (and, if short, replaced)
% above; a second, whole-mesh fallback would silently double-cover a strip that
% was already handled.
[H.fwHinge, H.fwPlate, ~, rtF, flapBox, flapTris] = localSurface(world, ...
    H.root, Pf, hingeA, localIso(hingeA) + Tb, M.body.upAxisFix, ...
    'WingFWHinge', 'WingFW', '', +1, M.band*sc, BAND_INSET, ...
    BAND_T, NBIN_FW, palFW(3,:), ...
    [min(fbR(1),fbL(1)) max(fbR(2),fbL(2)) min(fbR(3),fbL(3)) max(fbR(4),fbL(4)) ...
     min(fbR(5),fbL(5)) max(fbR(6),fbL(6))], 1);
rt = max(rt, rtF);

% Two station bands, one per strip, each riding that strip's OWN lower/forward
% edge (edgeSign +1, same sense on both sides -- the sides differ only in x,
% not in which edge deploys) -- children of the merged plate actor, in the same
% hinge-centred asset frame note 1 established for every other child here.
% NBIN_FW, not NBIN: these two ride a 40 cm strip, not the wing's 2 m span.
[VfR, FfR] = localPieceUnion(PfR);  VfR = VfR - hingeA;
[BvR, BnR, BfR] = localBandStrip(VfR, FfR, +1, M.band*sc, BAND_INSET, BAND_T, NBIN_FW);
bandR = localIsoActor(world, 'WingFWBandR', H.fwPlate);
rt = max(rt, localMeshWrite(bandR, BvR, BnR, BfR, [], []));
bandR.Color = palFW(3,:);  bandR.Metallic = 0;  bandR.Shininess = 0.35;  bandR.Specular = 0.2;

[VfL, FfL] = localPieceUnion(PfL);  VfL = VfL - hingeA;
[BvL, BnL, BfL] = localBandStrip(VfL, FfL, +1, M.band*sc, BAND_INSET, BAND_T, NBIN_FW);
bandL = localIsoActor(world, 'WingFWBandL', H.fwPlate);
rt = max(rt, localMeshWrite(bandL, BvL, BnL, BfL, [], []));
bandL.Color = palFW(3,:);  bandL.Metallic = 0;  bandL.Shininess = 0.35;  bandL.Specular = 0.2;

H.fwBand  = [bandR bandL];          % 1x2: unrealPlayback/Pose loop over this
H.fwBands = H.fwBand;               % same array, plural alias (plan spec 4)

[deployDeg, tuckDeg, fcn] = localHingeMap();
H.fwHingeDeg = fcn;
H.fwHinge.Rotation = [0 tuckDeg 0];                 % delivered state = flush

% =========================================================================
% 4. THE WING: the Agil's plane and struts, transplanted onto the Tur deck
% =========================================================================
donor = localIsoActor(world, 'DonorSrc', H.root);
load(donor, M.donor.file, scD);
donor.Rotation = M.donor.upAxisFix;
donor.Hidden = true;
LD = localWalk(donor, [0 0 0], '');
if isempty(LD)
    error('zenvoCarActors:emptyDonor', ...
        'zenvoCarActors: %s loaded no geometry.', M.donor.file);
end
wb = M.wingBox  * scD;
pb = M.pylonBox * scD;
% PRECEDENCE: the pylon box overlaps the wing box where the strut meets the
% plane, and the manifest gives the +x strut only, so it is applied to BOTH signs
% of x and it wins there.
pbBoth = [-pb(2) pb(2) pb(3:6)];
[Pp, nCullP] = localExtract(world, [], LD, ...
    @(c) localInBox([abs(c(:,1)) c(:,2:3)], pb), pbBoth, [], [], '');
[Pw2, nCullW] = localExtract(world, [], LD, ...
    @(c) localInBox(c, wb) & ~localInBox([abs(c(:,1)) c(:,2:3)], pb), wb, [], [], '');

pivotA   = mean(M.wingPivot, 1) * scD;
place    = M.wingPlace * sc;
pivotISO = localIso(pivotA + place) + Tb;
[H.rwPlate, H.rwPlane, H.rwBand, rtR, wingBox, wingTris] = localSurface(world, ...
    H.root, Pw2, pivotA, pivotISO, M.donor.upAxisFix, ...
    'WingRW', 'WingRWPlane', 'WingRWBand', -1, M.band*scD, BAND_INSET, ...
    BAND_T, NBIN, palRW(1,:), wb, MIN_TRIS);
rt = max(rt, rtR);

% SKIRT. A strut comes out of the box wearing a ring of the Agil's own deck skin
% around its foot -- measured, 48 mm of it below the box floor. Faces lying
% ENTIRELY below the floor are pure skirt and are dropped; the ones that straddle
% it stay, because they are the foot. What is left below the cut is buried inside
% the Tur's deck, where nothing can see it.
Pp = localTrimBelow(Pp, pb(5));
pylonTris = localTotalPieceTris(Pp);
if pylonTris < MIN_TRIS
    warning('zenvoCarActors:pylonFallback', ...
        ['zenvoCarActors: the pylon box caught only %d triangles, so the struts ' ...
         'are a PLAIN PLATE of the box''s size, not the asset''s own geometry. ' ...
         'Re-measure M.pylonBox.'], pylonTris);
    Pp = localPlatePieces(pb);
    pylonTris = localTotalPieceTris(Pp);
end
[H.rwPylons, ~, rtY, Pv] = localBuildParts(world, H.root, 'WingRWPylons', Pp, ...
                                           pivotA, M.donor.upAxisFix);
H.rwPylons.Translation = pivotISO;
rt = max(rt, rtY);
Pv = Pv + pivotA;                                   % back to the donor asset frame
pylonBox = reshape([min(Pv,[],1); max(Pv,[],1)], 1, []);

% THE FEET. Measured here on the Tur's own LOADED leaves, not on the manifest's
% M.deckZ, which is only the starting point the offset was derived from. The walk
% is over the original leaves rather than the rebuilt ones, which is exact in this
% footprint and only in this footprint: nothing under the pylon feet is split or
% hidden, so the leaves and the drawn surface are the same geometry there. (The
% three leaves this build DOES replace are at the nose, 4 m away, and the wheel
% leaves it hides are nowhere near.)
%
% WHAT "FOOT" MEANS, and it is not the obvious thing: it is the manifest's CUT
% LINE, M.pylonFeetZ, where the struts were sliced out of the donor -- NOT the
% drawn mesh's lowest vertex. A strut comes out of its box wearing a skirt of the
% Agil's own deck skin, so the mesh reaches ~21 mm further down and the struts are
% drawn planted THROUGH the deck skin rather than resting on it. That is
% deliberate (a buried foot is invisible, a floating one is not) and it is why
% both numbers are reported: feetZ is the cut line the tolerance applies to,
% feetZMesh is where the drawn geometry actually stops.
deckMeas  = localDeckAt(L, pb(1:2), pb(3:4) + place(2), 1.05*sc);
feetZ     = M.pylonFeetZ*scD + place(3);
feetZMesh = min(Pv(:,3)) + place(3);
feetGap   = feetZ - deckMeas;
if ~isfinite(deckMeas)
    error('zenvoCarActors:deck', ...
        ['zenvoCarActors: no Tur geometry under the pylon feet (|x| %.3f..%.3f, ' ...
         'y %.3f..%.3f). M.pylonBox does not describe struts that stand on this ' ...
         'body.'], pb(1), pb(2), pb(3), pb(4));
end
if abs(feetGap) > 0.01
    error('zenvoCarActors:feet', ...
        ['zenvoCarActors: the pylon feet sit at z = %.4f m and the Tur deck under ' ...
         'them measures %.4f m, so the wing %s by %.1f mm. Set M.deckZ = %.4f and ' ...
         'M.wingPlace(3) = %.4f (= deckZ - pylonFeetZ) in the manifest.'], ...
        feetZ, deckMeas, localFloatWord(feetGap), 1000*abs(feetGap), ...
        deckMeas, deckMeas - M.pylonFeetZ*scD);
end

% The donor must not render. remove() if the API has it, a hidden subtree if not.
donorRemoved = false;
try
    remove(world, donor);
    donorRemoved = true;
catch
    localHideSubtree(donor);
end

% =========================================================================
% 5. CAMERA RIG AND BOOKKEEPING
% =========================================================================
H.camRig = localIsoActor(world, 'CamRig', H.root);
H.camRig.Translation = [4.9 -2.7 0.92];
H.camRig.Rotation    = [0 3.5 151.2];

wbISO    = localIso(M.wheelHub.FL*sc) - localIso(M.wheelHub.RL*sc);
trkDrawn = abs(M.wheelHub.FL(1) - M.wheelHub.FR(1)) * sc;

H.palRW = palRW;  H.palFW = palFW;
H.stats = struct( ...
    'bodyTris',      localTotalTris(L), ...
    'wingTris',      wingTris, ...
    'pylonTris',     pylonTris, ...
    'flapTris',      flapTris, ...
    'flapTrisR',     trisR, ...
    'flapTrisL',     trisL, ...
    'culledNose',    nCullNose, ...
    'culledDonor',   nCullP + nCullW, ...
    'meshRoundTrip', rt, ...
    'wheelbase',     wbISO(1), ...
    'track',         trkDrawn, ...
    'nSkin',         nSkin, ...
    'nLeaves',       numel(L), ...
    'nDonorLeaves',  numel(LD), ...
    'nWheelLeaves',  nWheelLeaf, ...
    'wheelR',        wheelR, ...
    'wheelCentreOff', wheelOff, ...
    'bodyLength',    bbAll(2,2) - bbAll(1,2), ...
    'bodyWidth',     bbAll(2,1) - bbAll(1,1), ...
    'bodyHeight',    bbAll(2,3), ...
    'bodyTranslation', Tb, ...
    'scale',         sc, ...
    'flapBox',       flapBox, ...
    'flapBoxR',      fbR, ...
    'flapBoxL',      fbL, ...
    'wingBox',       wingBox, ...
    'pylonBox',      pylonBox, ...
    'deckZMeasured', deckMeas, ...
    'feetZ',         feetZ, ...
    'feetZMesh',     feetZMesh, ...
    'feetGap',       feetGap, ...
    'donorRemoved',  donorRemoved, ...
    'fwDeployDeg',   deployDeg, ...
    'fwTuckDeg',     tuckDeg);
varargout = {H};
end

% =========================================================================
% FRAMES
% =========================================================================
function q = localIso(p)
%LOCALISO Asset point -> the ISO offset of the same point on an actor that
% carries upAxisFix as its Rotation. Measured by render (note 1 in the header):
% the asset's long axis becomes ISO x and the asset's +x becomes ISO +y. The map
% has determinant -1 because the engine's own vertex y flip is folded into it,
% which is why a yaw of +90 and "ISO y = the asset's +x" are both true at once.
q = [p(:,2) p(:,1) p(:,3)];
end

function w = localFloatWord(gap)
if gap > 0, w = 'floats'; else, w = 'sinks'; end
end

% =========================================================================
% ACTORS AND MESH ARITHMETIC. Faces are 0-BASED in and out.
% =========================================================================
function a = localIsoActor(world, name, parent)
%LOCALISOACTOR One Movable actor with ISO8855 set BEFORE it joins the world.
% 'Default' is radians and the other lateral sign, so the CoordinateSystem write
% has to happen before add/load or the actor is silently mis-posed.
a = sim3d.Actor(ActorName = name, Mobility = sim3d.utils.MobilityTypes.Movable);
a.CoordinateSystem = 'ISO8855';
if isempty(parent), add(world, a); else, add(world, a, parent); end
end

function rt = localMeshWrite(actor, V, N, F, TC, C)
%LOCALMESHWRITE createMesh in the engine's measured convention, then READ THE
% RESULT BACK and refuse to continue if it is not what was asked for.
%
% Measured by render 2026-09-17 (simulink/work/zm1_top.png): createMesh on an
% ISO8855 actor renders AT the vertices it is given, so the asset vertices go
% straight through with their original winding; the getters then hand back
% [V(:,1) -V(:,2) V(:,3)] and F(:,[1 3 2]). The read-back is compared against
% THAT, which is what makes this a mirror guard rather than a tautology: a
% re-introduced mirror is invisible to every other check in this file -- a
% mirrored wheel is still centred on its hub, still the right radius, still the
% right vertex count -- so comparing what came back against what went in is the
% only thing that catches it.
createMesh(actor, V, N, F, TC, C);
Vb = actor.Vertices;  Fb = actor.Faces;
if ~isequal(size(Vb), size(V)) || ~isequal(size(Fb), size(F))
    error('zenvoCarActors:meshRoundTrip', ...
        ['zenvoCarActors: %s came back %dx%d verts / %dx%d faces after createMesh, ' ...
         'not the %dx%d / %dx%d that were written.'], char(actor.Name), ...
        size(Vb), size(Fb), size(V), size(F));
end
Vexp = [V(:,1) -V(:,2) V(:,3)];
rt = max(abs(Vb(:) - Vexp(:)));
if isempty(rt), rt = 0; end
if rt > 1e-6 || ~isequal(Fb, F(:,[1 3 2]))
    error('zenvoCarActors:meshRoundTrip', ...
        ['zenvoCarActors: %s does not read back as it was written (max vertex ' ...
         'residual %.3g m, winding %s). The engine''s ISO8855 mesh convention has ' ...
         'moved: the copy is mirrored or re-wound relative to its source.'], ...
        char(actor.Name), rt, string(isequal(Fb, F(:,[1 3 2]))));
end
end

function [V2, N2, F2, keep] = localCompact(V, N, F)
%LOCALCOMPACT Drop vertices no surviving face refers to, and remap the faces.
% Without this a cut part's vertices stay in the rebuilt buffer and in its
% bounding box, unreferenced but measurable.
if isempty(F)
    V2 = zeros(0,3); N2 = zeros(0,3); F2 = zeros(0,3); keep = []; return
end
[keep, ~, back] = unique(F(:) + 1);
V2 = V(keep,:);  N2 = N(keep,:);
F2 = reshape(back, size(F)) - 1;
end

function [V, N, F] = localBoxMesh(c, s)
%LOCALBOXMESH An axis-aligned box in the asset frame, 8 verts / 12 tris.
h = s(:).'/2;
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] .* h + c(:).';
N = (V - c(:).') ./ max(vecnorm(V - c(:).', 2, 2), eps);
F = [0 1 2; 0 2 3; 4 6 5; 4 7 6; 0 4 5; 0 5 1; 1 5 6; 1 6 2; ...
     2 6 7; 2 7 3; 3 7 4; 3 4 0];
end

% =========================================================================
% PIECES: geometry plus the material it has to keep
% =========================================================================
function P = localPiece(V, N, F, TC, a)
%LOCALPIECE One chunk of extracted geometry tagged with its source material.
% The key is what decides which chunks can share an actor: an actor carries ONE
% texture, so chunks with different textures cannot be merged without losing one.
tex = char(a.Texture);
P = struct('V', V, 'N', N, 'F', F, 'TC', TC, ...
    'key', sprintf('%s|%s|%.4f|%.4f|%.4f|%d', tex, mat2str(a.Color, 5), ...
                   a.Metallic, a.Shininess, a.Specular, a.TwoSided), ...
    'src', struct('Texture', tex, 'Color', a.Color, 'Metallic', a.Metallic, ...
                  'Shininess', a.Shininess, 'Specular', a.Specular, ...
                  'TwoSided', a.TwoSided));
end

function P = localLeafPieces(L, sel)
%LOCALLEAFPIECES Whole leaves, in the asset frame, as pieces.
idx = find(sel);  P = {};
for j = 1:numel(idx)
    r = L{idx(j)};
    [V, N, F] = localLeafGeom(r);
    if isempty(F), continue, end
    P{end+1} = localPiece(V, N, F, r.actor.TextureCoordinates, r.actor); %#ok<AGROW>
end
end

function P = localPlatePieces(box)
%LOCALPLATEPIECES The spec-5 fallback: a plain mirrored pair of plates.
c = (box([1 3 5]) + box([2 4 6]))/2;
s = max(box([2 4 6]) - box([1 3 5]), [0.02 0.02 0.006]);
[V, N, F] = localBoxMesh(c, s);
a = struct('Texture', '', 'Color', [0.05 0.05 0.05], 'Metallic', 0.3, ...
           'Shininess', 0.3, 'Specular', 0.3, 'TwoSided', false);
P = {localPiece(V, N, F, [], a), ...
     localPiece([-V(:,1) V(:,2:3)], [-N(:,1) N(:,2:3)], F(:,[1 3 2]), [], a)};
end

function P = localTrimBelow(P, zFloor)
%LOCALTRIMBELOW Drop faces lying ENTIRELY below a z, piece by piece.
for i = 1:numel(P)
    F = P{i}.F;  V = P{i}.V;
    if isempty(F), continue, end
    gone = all(reshape(V(F+1, 3), size(F)) < zFloor, 2);
    if ~any(gone), continue, end
    [V2, N2, F2, keep] = localCompact(V, P{i}.N, F(~gone,:));
    P{i}.V = V2;  P{i}.N = N2;  P{i}.F = F2;
    if ~isempty(P{i}.TC), P{i}.TC = P{i}.TC(keep,:); end
end
end

function n = localTotalPieceTris(P)
n = 0;
for i = 1:numel(P), n = n + size(P{i}.F,1); end
end

function [V, F] = localPieceUnion(P)
%LOCALPIECEUNION Every piece's geometry in one buffer, for bboxes and bands.
Vc = cell(numel(P),1); Fc = Vc; n0 = 0;
for i = 1:numel(P)
    Vc{i} = P{i}.V;  Fc{i} = P{i}.F + n0;  n0 = n0 + size(P{i}.V,1);
end
V = vertcat(Vc{:});  F = vertcat(Fc{:});
if isempty(V), V = zeros(0,3); F = zeros(0,3); end
end

function [prim, acts, rt, V, F] = localBuildParts(world, parent, name, P, centre, rotDeg)
%LOCALBUILDPARTS Rebuild a set of pieces as ONE ACTOR PER MATERIAL, re-centred.
%
% The largest material takes the frozen name and the body's own yaw as its
% Rotation; the rest hang off it as children with no transform of their own, so
% their vertices stay in the same asset frame and the engine places them
% identically (note 1 in the header). That is what keeps four different textures
% -- tyre, calliper, carbon, paint -- on parts the plan names as single actors.
[V, F] = localPieceUnion(P);
V = V - centre;
keys = cell(numel(P),1);
for i = 1:numel(P), keys{i} = P{i}.key; end
[uk, ia] = unique(keys, 'stable');
nV = zeros(numel(uk),1);
for g = 1:numel(uk)
    for i = 1:numel(P)
        if strcmp(keys{i}, uk{g}), nV(g) = nV(g) + size(P{i}.V,1); end
    end
end
[~, ord] = sort(nV, 'descend');
acts = cell(numel(uk),1);  rt = 0;  prim = [];
for gi = 1:numel(ord)
    g = ord(gi);
    [Vg, Ng, Fg, TCg] = localCatPieces(P(strcmp(keys, uk{g})), centre);
    if gi == 1
        a = localIsoActor(world, name, parent);
        a.Rotation = rotDeg;
        prim = a;
    else
        a = localIsoActor(world, sprintf('%sM%d', name, gi), prim);
    end
    rt = max(rt, localMeshWrite(a, Vg, Ng, Fg, TCg, []));
    src = P{ia(g)}.src;
    a.Color = src.Color;  a.Metallic = src.Metallic;
    a.Shininess = src.Shininess;  a.Specular = src.Specular;
    a.TwoSided = src.TwoSided;
    if ~isempty(src.Texture) && ~isempty(TCg)
        a.Texture = src.Texture;
    end
    acts{gi} = a;
end
end

function [V, N, F, TC] = localCatPieces(P, centre)
Vc = cell(numel(P),1); Nc = Vc; Fc = Vc; Tc = Vc; n0 = 0; okTC = true;
for i = 1:numel(P)
    Vc{i} = P{i}.V - centre;  Nc{i} = P{i}.N;  Fc{i} = P{i}.F + n0;
    if isempty(P{i}.TC) || size(P{i}.TC,1) ~= size(P{i}.V,1)
        okTC = false;
    else
        Tc{i} = P{i}.TC;
    end
    n0 = n0 + size(P{i}.V,1);
end
V = vertcat(Vc{:});  N = vertcat(Nc{:});  F = vertcat(Fc{:});
if okTC, TC = vertcat(Tc{:}); else, TC = []; end
end

% =========================================================================
% TREE WALKING. Leaves are reached only by walking Children: the auto-assigned
% ActorNN names are a per-world counter and are not stable between loads.
% =========================================================================
function L = localWalk(actor, T0, path)
%LOCALWALK Every geometry leaf under a subtree, with its accumulated translation
% already added to its bounding box and its ancestor PATH recorded (the only
% thing that separates a rotor leaf from a calliper leaf on this asset).
% Vertices are NOT cached: 980 leaves of them is most of a car, and only the few
% leaves that touch a box are ever read in full (localLeafGeom).
L = {};
ch = actor.Children;                               % SNAPSHOT: lazily rebuilt
if ~isstruct(ch), return, end
nm = fieldnames(ch);
for i = 1:numel(nm)
    kid = ch.(nm{i});
    Tk  = T0 + kid.Translation;
    pk  = [path '/' char(kid.Name)];
    V   = kid.Vertices;
    if ~isempty(V)
        bb = [min(V,[],1) + Tk; max(V,[],1) + Tk];
        L{end+1} = struct('actor', kid, 'name', char(kid.Name), 'path', pk, ...
            'T', Tk, 'bb', bb, 'nTri', size(kid.Faces,1)); %#ok<AGROW>
    end
    L = [L, localWalk(kid, Tk, pk)]; %#ok<AGROW>
end
end

function [V, N, F] = localLeafGeom(rec)
%LOCALLEAFGEOM One leaf's geometry in the ASSET frame.
a = rec.actor;
V = a.Vertices + rec.T;  N = a.Normals;  F = a.Faces;
end

function bb = localUnionBox(L)
bb = [Inf Inf Inf; -Inf -Inf -Inf];
for i = 1:numel(L)
    bb(1,:) = min(bb(1,:), L{i}.bb(1,:));
    bb(2,:) = max(bb(2,:), L{i}.bb(2,:));
end
end

function n = localTotalTris(L)
n = 0;
for i = 1:numel(L), n = n + L{i}.nTri; end
end

function localHideSubtree(actor)
actor.Hidden = true;
ch = actor.Children;
if isstruct(ch)
    nm = fieldnames(ch);
    for i = 1:numel(nm), localHideSubtree(ch.(nm{i})); end
end
end

function in = localInBox(V, b)
in = V(:,1) >= b(1) & V(:,1) <= b(2) & V(:,2) >= b(3) & V(:,2) <= b(4) & ...
     V(:,3) >= b(5) & V(:,3) <= b(6);
end

function t = localBoxOverlap(bb, b)
t = bb(2,1) >= b(1) && bb(1,1) <= b(2) && bb(2,2) >= b(3) && bb(1,2) <= b(4) && ...
    bb(2,3) >= b(5) && bb(1,3) <= b(6);
end

% =========================================================================
% EXTRACTION
% =========================================================================
function [P, nCull, nSkin, rt] = localExtract(world, parent, L, inFcn, preBox, ...
                                              rotDeg, Tb, skinName)
%LOCALEXTRACT Split every leaf that has faces the predicate claims, and (when
% skinName is given) REBUILD each split leaf without them so the part leaves a
% real hole behind it.
%
% The test is on the face's CENTROID, which makes this a genuine PARTITION: every
% face goes to exactly one side, so the part and the hole are complementary --
% never doubled (the plan forbids that outright) and never separated by a gap. A
% vertex-wise "any inside" rule would do neither: measured on this asset it
% dragged the flap 93 mm past the back of its own box on single long bumper
% triangles, and cut a correspondingly oversized hole. The rebuilt leaf keeps its
% own texture, texture coordinates and material -- that is what stops the nose
% slot from repainting the car -- and the original is hidden.
% With skinName empty (the donor side) nothing is rebuilt: the donor is removed
% from the world wholesale a few lines later.
P = {};  nCull = 0;  nSkin = 0;  rt = 0;
for i = 1:numel(L)
    % Cheap bbox reject FIRST: reading 2000 leaves' vertices twice is most of a
    % minute, and only a handful of them ever touch one of these boxes.
    if ~localBoxOverlap(L{i}.bb, preBox), continue, end
    [Vl, Nl, Fl] = localLeafGeom(L{i});
    if isempty(Fl), continue, end
    cen  = (Vl(Fl(:,1)+1, :) + Vl(Fl(:,2)+1, :) + Vl(Fl(:,3)+1, :)) / 3;
    take = inFcn(cen);
    if ~any(take), continue, end
    nCull = nCull + nnz(take);
    TCl = L{i}.actor.TextureCoordinates;
    [V2, N2, F2, keep] = localCompact(Vl, Nl, Fl(take,:));
    TC2 = [];
    if ~isempty(TCl) && size(TCl,1) == size(Vl,1), TC2 = TCl(keep,:); end
    P{end+1} = localPiece(V2, N2, F2, TC2, L{i}.actor); %#ok<AGROW>
    if isempty(skinName), continue, end
    nSkin = nSkin + 1;
    rt = max(rt, localRebuildLeaf(world, parent, L{i}, Vl, Nl, Fl(~take,:), TCl, ...
                                  sprintf('%s%02d', skinName, nSkin), rotDeg, Tb));
    L{i}.actor.Hidden = true;
end
end

function [Ps, nCull, nSkin, rt] = localExtractMulti(world, parent, L, boxes, ...
                                                     rotDeg, Tb, skinName)
%LOCALEXTRACTMULTI Like localExtract, but against a LIST of boxes in ONE pass.
%
% This exists because a leaf here can be big enough to touch MORE than one box
% at once -- measured on the flap strips: the Tur's broad "Base Coloured" body
% panel and its carbon lower fascia both span nearly the car's whole width, so
% each is a single leaf whose bbox overlaps BOTH the right and the left strip
% box. Calling localExtract once per box independently would rebuild THAT SAME
% leaf twice, each rebuild only missing its OWN box's faces and still showing
% the OTHER box's faces as present -- two overlapping, partially-doubled skins
% for one leaf, exactly the doubled surface the plan forbids. Here every leaf
% is read ONCE, the faces claimed by ANY box are removed in ONE rebuild, and
% the claimed faces are then split by WHICH box(es) claimed them into Ps{k},
% k = 1..numel(boxes) -- so a caller building N surfaces from N boxes still
% gets N clean, non-overlapping piece lists.
nB = numel(boxes);
Ps = cell(1, nB);
for k = 1:nB, Ps{k} = {}; end
nCull = 0;  nSkin = 0;  rt = 0;
for i = 1:numel(L)
    hit = false(1, nB);
    for k = 1:nB, hit(k) = localBoxOverlap(L{i}.bb, boxes{k}); end
    if ~any(hit), continue, end
    [Vl, Nl, Fl] = localLeafGeom(L{i});
    if isempty(Fl), continue, end
    cen = (Vl(Fl(:,1)+1,:) + Vl(Fl(:,2)+1,:) + Vl(Fl(:,3)+1,:)) / 3;
    takeK = false(size(Fl,1), nB);
    for k = 1:nB
        if hit(k), takeK(:,k) = localInBox(cen, boxes{k}); end
    end
    takeAny = any(takeK, 2);
    if ~any(takeAny), continue, end
    % PARTITION the claimed faces, don't just test membership per box: a face
    % inside two boxes at once goes to the FIRST box that claims it, so no face
    % can be drawn twice (once in each output surface) while being removed from
    % the source only once below. The shipped flapBoxes never overlap
    % (test_carAsset [1] forbids it), so this is defensive -- but this helper is
    % written for any box list, and a silently doubled face is exactly what the
    % single-rebuild-per-leaf design above exists to prevent.
    takeK = takeK & cumsum(takeK, 2) == 1;
    nCull = nCull + nnz(takeAny);
    TCl = L{i}.actor.TextureCoordinates;
    for k = 1:nB
        if ~any(takeK(:,k)), continue, end
        [V2, N2, F2, keep] = localCompact(Vl, Nl, Fl(takeK(:,k),:));
        TC2 = [];
        if ~isempty(TCl) && size(TCl,1) == size(Vl,1), TC2 = TCl(keep,:); end
        Ps{k}{end+1} = localPiece(V2, N2, F2, TC2, L{i}.actor);
    end
    if isempty(skinName), continue, end
    nSkin = nSkin + 1;
    rt = max(rt, localRebuildLeaf(world, parent, L{i}, Vl, Nl, Fl(~takeAny,:), TCl, ...
                                  sprintf('%s%02d', skinName, nSkin), rotDeg, Tb));
    L{i}.actor.Hidden = true;
end
end

function rt = localRebuildLeaf(world, parent, rec, Vl, Nl, Fkeep, TCl, name, rotDeg, Tb)
%LOCALREBUILDLEAF One loaded leaf, minus the faces that were split off it, as a
% new actor that keeps the original's look. The new actor hangs off the ROOT with
% the body's own yaw as its Rotation -- exactly like the wheel copies -- rather
% than off Body, so it goes through the one placement convention this file
% measured instead of a second, unverified one.
a = rec.actor;
[V2, N2, F2, keep] = localCompact(Vl, Nl, Fkeep);
TC2 = [];
if ~isempty(TCl) && size(TCl,1) == size(Vl,1), TC2 = TCl(keep,:); end
nw = localIsoActor(world, name, parent);
rt = localMeshWrite(nw, V2, N2, F2, TC2, []);
nw.Color = a.Color;  nw.Metallic = a.Metallic;  nw.Shininess = a.Shininess;
nw.Specular = a.Specular;  nw.TwoSided = a.TwoSided;  nw.Transparency = a.Transparency;
if ~isempty(char(a.Texture)) && ~isempty(TC2)
    nw.Texture = a.Texture;
end
nw.Rotation = rotDeg;  nw.Translation = Tb;
end

% =========================================================================
% ACTUATED SURFACES
% =========================================================================
function [pivot, plate, band, rt, bbox, nTri] = localSurface(world, root, P, ...
        axisA, axisISO, rotDeg, pivotName, plateName, bandName, edgeSign, ...
        chord, inset, thick, nBin, chip, box, minTris)
%LOCALSURFACE One actuated surface: a pivot actor the caller rotates, the
% extracted mesh under it, and (unless bandName is empty) a station-colour band
% on its free edge.
%
% The pivot has to be the PARENT of the mesh because the two rotations do not
% commute: the mesh is yawed onto ISO by upAxisFix first and pitched about the
% pivot line second. The band is a child of the MESH actor with no rotation of
% its own, so its vertices live in the same asset frame as the mesh's and no
% second frame convention is needed anywhere.
%
% bandName == '' skips the single combined band entirely and returns band = [];
% the flap builds its own two per-strip bands after this returns instead (a
% single edge-following ribbon over BOTH strips at once would cut straight
% across the gap between them).
nTri = localTotalPieceTris(P);
if nTri < minTris
    % Plan spec 5: never leave a doubled surface. The box has already been cut
    % out of the parent body by the time this runs, so the fallback replaces it.
    warning('zenvoCarActors:plateFallback', ...
        ['zenvoCarActors: %s came out of its box with only %d triangles, so it is ' ...
         'a PLAIN PLATE of the box''s size instead of the asset''s own geometry. ' ...
         'Re-measure the box in the manifest.'], plateName, nTri);
    P = localPlatePieces(box);
    nTri = localTotalPieceTris(P);
end

pivot = localIsoActor(world, pivotName, root);
pivot.Translation = axisISO;
[plate, ~, rt, V, F] = localBuildParts(world, pivot, plateName, P, axisA, rotDeg);
bbox = reshape([min(V,[],1); max(V,[],1)], 1, []) + repelem(axisA, 1, 2);

if isempty(bandName)
    band = [];
    return
end
[Bv, Bn, Bf] = localBandStrip(V, F, edgeSign, chord, inset, thick, nBin);
band = localIsoActor(world, bandName, plate);
rt = max(rt, localMeshWrite(band, Bv, Bn, Bf, [], []));
band.Color = chip;  band.Metallic = 0;  band.Shininess = 0.35;  band.Specular = 0.2;
end

function [V, N, F] = localBandStrip(Vp, Fp, edgeSign, chord, inset, thick, nBin)
%LOCALBANDSTRIP A chord-wide colour band that FOLLOWS a swept edge, as ONE
% continuous ribbon.
%
% edgeSign -1 takes the most-negative-y edge (the wing's trailing edge, ISO -x)
% and rides the section's mid-height; +1 takes the most-positive-y edge (the
% flap's lower, forward edge) and rides the section's underside. Both edges are
% SWEPT -- the Agil's wing trailing edge moves 0.14 m between the centreline and
% the tip -- so the band is built station by station across the span and the
% stations are stitched into one closed tube. Building it as a row of separate
% boxes (the first attempt, simulink/work/zb1_top.png) drew a dotted line of blue
% blocks with daylight between them, which is what these renders are for.
used = unique(Fp(:) + 1);
P = Vp(used,:);
x0 = min(P(:,1)) + inset/2;  x1 = max(P(:,1)) - inset/2;
if x1 <= x0, x0 = min(P(:,1)); x1 = max(P(:,1)); end
xs = linspace(x0, x1, nBin+1);
half = max(x1 - x0, 0.02) / nBin;             % lateral reach of one station
ring = zeros(4, 3, numel(xs));  ok = false(numel(xs),1);
for i = 1:numel(xs)
    s = abs(P(:,1) - xs(i)) <= half;
    if nnz(s) < 3, continue, end
    Q = P(s,:);
    if edgeSign < 0
        yE = min(Q(:,2));  yLo = yE;  yHi = yE + chord;
    else
        yE = max(Q(:,2));  yLo = yE - chord;  yHi = yE;
    end
    zc = Q(abs(Q(:,2) - yE) <= chord, 3);
    if isempty(zc), zc = Q(:,3); end
    if edgeSign < 0, z0 = median(zc); else, z0 = min(zc); end
    ring(:,:,i) = [xs(i) yLo z0-thick/2
                   xs(i) yHi z0-thick/2
                   xs(i) yHi z0+thick/2
                   xs(i) yLo z0+thick/2];
    ok(i) = true;
end
idx = find(ok);
if numel(idx) < 2
    % Nothing to ride: one plain box across the whole span still reads as a band.
    if edgeSign < 0
        yE = min(P(:,2));  yLo = yE;  yHi = yE + chord;
    else
        yE = max(P(:,2));  yLo = yE - chord;  yHi = yE;
    end
    [V, N, F] = localBoxMesh([(x0+x1)/2, (yLo+yHi)/2, (min(P(:,3))+max(P(:,3)))/2], ...
                             [max(x1-x0, 0.02), yHi-yLo, thick]);
    return
end
V = reshape(permute(ring(:,:,idx), [1 3 2]), [], 3);
n = numel(idx);
F = zeros(8*(n-1) + 4, 3);
r = 0;
for j = 1:n-1
    b0 = (j-1)*4;  b1 = j*4;
    for e = 0:3
        a1 = b0 + e;  a2 = b0 + mod(e+1,4);
        c1 = b1 + e;  c2 = b1 + mod(e+1,4);
        F(r+1,:) = [a1 a2 c2];  F(r+2,:) = [a1 c2 c1];  r = r + 2;
    end
end
e0 = 0;  e1 = (n-1)*4;                          % end caps
F(r+1,:) = [e0+0 e0+2 e0+1];  F(r+2,:) = [e0+0 e0+3 e0+2];
F(r+3,:) = [e1+0 e1+1 e1+2];  F(r+4,:) = [e1+0 e1+2 e1+3];
c = (max(V,[],1) + min(V,[],1))/2;
N = (V - c) ./ max(vecnorm(V - c, 2, 2), eps);
end

% =========================================================================
function z = localDeckAt(L, xBand, yBand, zCap)
%LOCALDECKAT The top of the Tur's own bodywork in the pylon feet's footprint.
% Both signs of x, because the struts are a mirrored pair. zCap keeps anything
% well above the deck out of it; NaN if the footprint is empty.
z = NaN;
for i = 1:numel(L)
    bb = L{i}.bb;
    % |x| interval of this leaf: a leaf that straddles the centreline reaches
    % |x| = 0, which a signed bbox test would get wrong.
    if bb(1,1) <= 0 && bb(2,1) >= 0, axLo = 0; else, axLo = min(abs(bb(:,1))); end
    axHi = max(abs(bb(:,1)));
    if axHi < xBand(1) || axLo > xBand(2), continue, end
    if bb(2,2) < yBand(1) || bb(1,2) > yBand(2) || bb(1,3) > zCap, continue, end
    V = L{i}.actor.Vertices + L{i}.T;
    s = abs(V(:,1)) >= xBand(1) & abs(V(:,1)) <= xBand(2) & ...
        V(:,2) >= yBand(1) & V(:,2) <= yBand(2) & V(:,3) <= zCap;
    if any(s), z = max([z; V(s,3)]); end
end
end

% =========================================================================
function [deployDeg, tuckDeg, fcn] = localHingeMap()
%LOCALHINGEMAP The front flap's two end angles and the map between them.
% SINGLE OWNER of both numbers, reachable without a world through the static
% query form zenvoCarActors('hingeMap'), which is how the live route's Pose
% function and test_carAsset [7] get them without starting the engine.
%
% TUCK IS ZERO BY CONSTRUCTION, and that is the point of cutting the flap out of
% the car instead of modelling one: at 0 the band is exactly where it was cut
% from, so the low-downforce station is the delivered bumper, flush, with no seam
% to hide. Deploy swings the band's free (lower, forward) edge down and out of its
% slot; the angle is the one that clears the bumper skin far enough for the
% station band on that edge to read, chosen on the nose renders
% simulink/work/zb*_nose_dep.png.
%
% CUT FROM 30 TO 15 in fix round 1 (2026-09-18), on the angle sweep
% simulink/work/t1bf_ang.m (zc5_uv_a30/a22/a16/a12/a08/a00, all READ). 30 deg
% was inherited from the old central-vent flap, whose extracted piece was only a
% few centimetres deep. The duct strip is 29 cm from the hinge line to its
% forward tip, so 30 deg drops that tip 145 mm -- out of its own duct, past the
% splitter, and into the frames as a row of disconnected slivers poking through
% whatever bodywork happened to be in front of it. 15 deg drops it 75 mm: still
% plainly open against the flush tucked state from the user's own vantage, but
% the strip stays in its slot and reads as a strip tilting rather than a shard
% swinging out.
deployDeg = 15;     % alphaFW =   0 : band swung down out of the bumper
tuckDeg   = 0;      % alphaFW = -25 : band flush, as delivered
fcn = @(aFW) tuckDeg + min(max((aFW + 25)/25, 0), 1) * (deployDeg - tuckDeg);
end

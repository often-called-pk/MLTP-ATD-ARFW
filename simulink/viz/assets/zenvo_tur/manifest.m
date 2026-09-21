function M = manifest()
%MANIFEST  2026 Zenvo Aurora Tur (Sketchfab, Ddiaz Design, CC BY-NC-SA 4.0) plus
% its rear-wing donor, the 2026 Zenvo Aurora Agil (same author, same licence) --
% the import recipe for the one car both render routes draw. zenvoCarActors reads
% ONLY this struct.
%
% MEASURED 2026-09-17 on the two shipped model.fbx files by
% simulink/tools/assetInspect.m and the throwaway probes t0_meas2/3/4 +
% t0_render in simulink/work (that folder is git-ignored, so every number that
% matters is repeated here with the measurement that produced it). Nothing below
% is a guess; where a number is a judgement call between two defensible cuts, the
% comment says so and says which way to move it.
%
% ============================ THE FRAME =================================
% Every coordinate in this file is in the ASSET (leaf-vertex) FRAME: what a
% loaded leaf's .Vertices returns, PLUS the accumulated Translations of the nodes
% above it. That sum matters -- 979 of the Tur's 980 geometry leaves carry a
% non-zero accumulated Translation, and reading .Vertices alone puts the wheels
% and the body in different frames (measured: the raw-vertex overall extent is
% 1.416 m tall, the root-frame one 1.147 m).
%
%   x  lateral      y  longitudinal, NOSE at +y      z  up
%
% The mesh is ALREADY GROUND-REFERENCED: the root-frame bbox is
% [-1.0143 -2.4015 0.0012] .. [1.0236 2.3963 1.1487], so the tyres stand on
% z = 0 and no groundLift term is needed (unlike the Alpine A480 this replaces).
%
% NOSE DIRECTION, and how it was settled. The file is authored nose-along-its-own
% +y, and M.body.upAxisFix yaws that onto ISO +x. The SIGN was determined by
% render, not by algebra (simulink/work/t0_tur2_front34.png and _rear34.png: with
% Rotation [0 0 90] the camera at ISO +x sees the nose, the camera at ISO -x sees
% the rear ZENVO panel and the diffuser). Eight independent geometric facts agree
% with that reading: the Agil's rear wing sits at leaf y -2.42..-1.85; the cabin
% (Interior_Geo_lodA_22) spans leaf y -0.40..+1.33, i.e. forward of centre; the
% seat belt is at leaf y -0.37..-0.06, behind it; the roof intake
% (GrilleNoAlpha3_Geo_lodA_16) is at leaf y -0.80..-0.52, behind the cabin; the
% bigger tyres (R 0.364 against 0.347) are on the leaf -y axle; the Agil's own
% node names put DWheel_Front_* on the leaf +y axle; the Tur's bumper vent band is
% at leaf y +2.04..+2.32; and the rear nameplate reads the right way round.
%
% (A 2026-09-17 spike note, simulink/work/zenvo_inspect.md, claimed the Agil's own
% Front/Rear node labels are physically SWAPPED. They are NOT -- that claim was a
% consequence of the same spike assuming the opposite nose direction. Do not
% "re-fix" the labels.)
%
% MARKER CONVENTION, measured the same day and needed by any builder that draws a
% box or copies a mesh (simulink/work/t0_agil2_side.png is the decisive frame,
% with the same box written both ways in two colours): on an ISO8855 actor,
%   createMesh(a, V, N, F, [], C)   RENDERS where a loaded leaf whose .Vertices
%                                   equal V renders -- pass V straight through,
%   a.Vertices                      then hands back [V(:,1) -V(:,2) V(:,3)].
% The flip lives in the GETTER, not in the renderer. A round-trip check that
% negates y before writing (as the deleted carAssetActors/localMeshWrite did)
% therefore passes while drawing the copy mirrored end-for-end on these y-long
% bodies. Write straight through and check the read-back against the negated
% vertices, or check it by render.
%
% ============================ WHAT IS WHERE =============================
% Both files arrive as ONE level-1 group, FBX_model_Transform, split by MATERIAL
% (not by part), with the wheels in their own subtree:
%   Tur   Body_lodA_0/{Badge Base Coloured GrilleAlpha1 GrilleNoAlpha2
%         GrilleNoAlpha3 InteriorColourZone Interior Kit0_Carbon1 LightEmissive
%         Light ManufacturerPlate SeatBelt WindowInside Window}_Geo_lodA_*,
%         body_Paint_Geo_lodABody_lodA_45, Wheel1A_3D_48/{Combined_Wheels_3D__49,
%         Calliper1_A_A_Clone__1053}                      980 leaves, 205367 tris
%   Agil  17 material groups + Wheel1A_3D_47/{DWheel_Front_L_48, DWheel_Front_R_341,
%         DWheel_Rear_L_633, DWheel_Rear_R_929, Calliper1_A_A_Clone__1225}
%                                                        1070 leaves, 420643 tris
% Consequence: neither the wing, the pylons nor the bumper vent band is a node.
% Each is a BOX of faces inside a shared shell, which is why this manifest ships
% boxes rather than node names for them.

M.slug = 'zenvo_tur';

% --- body: the Tur ------------------------------------------------------------
M.body.file  = 'model.fbx';                  % relative to this folder
% SCALE 1. Measured wheelbase 2.8020 m (front axle leaf y +1.3000, rear -1.5020,
% assetInspect corner clustering, simulink/work/inspect_tur2.log) against
% vp.l = 2.800 -- 2 mm, inside the 1 cm the plan allows, so no 2.8/wb shrink is
% applied and the drawn car keeps its own 4.798 m length (Zenvo workbook 4.819).
M.body.scale = 1;
% Yaw about z applied to the Body actor, degrees, ISO8855. +90 puts the file's
% own +y nose onto ISO +x -- render-verified, see THE FRAME above.
M.body.upAxisFix = [0 0 90];

% --- donor: the Agil, for the rear wing only ----------------------------------
M.donor.file  = fullfile('..', 'zenvo_agil', 'model.fbx');
% Measured wheelbase 2.8000 m exactly (front axle leaf y +1.2850, rear -1.5150),
% overall 2.028 x 4.849 x 1.141 m: the same base model as the Tur at the same
% scale, which is why M.wingPlace below is a few millimetres and not a fit.
M.donor.scale = 1;
M.donor.upAxisFix = [0 0 90];                % same file convention as the body

% --- wheels -------------------------------------------------------------------
% THE TUR HAS NO PER-CORNER WHEEL NODE. Unlike the Agil (four DWheel_* groups)
% and unlike the Alpine A480 this replaces, the Tur's export puts ALL FOUR
% corners' geometry under ONE instanced group per role -- so the four names below
% are deliberately IDENTICAL and are NOT what separates the corners. The corner
% split is GEOMETRIC: take the leaves whose own bbox centre lies within
% M.wheelGrabR of M.wheelHub.<corner>. Measured leaf counts with that rule:
% 235 / 235 / 247 / 247 (125 rotor + 110 or 122 stator each), and the four sets
% are disjoint and exhaust the wheel subtree.
M.wheelNodes = struct( ...
    'FL', 'Combined3DWheel_3DWheel_Front_L_Instance1_Src4_50', ...
    'FR', 'Combined3DWheel_3DWheel_Front_L_Instance1_Src4_50', ...
    'RL', 'Combined3DWheel_3DWheel_Front_L_Instance1_Src4_50', ...
    'RR', 'Combined3DWheel_3DWheel_Front_L_Instance1_Src4_50');
M.wheelGrabR = 0.45;                         % m, corner-assignment radius
% Union bbox centre of each corner's ROTOR leaves, asset frame, measured.
% CORNER LABELS ARE BY MEASURED POSITION, not by any name in the file: ISO x is
% the asset's +y (nose, see upAxisFix) and ISO y is taken as the asset's +x, the
% one convention here a render cannot pin down on a laterally symmetric body.
% Because each corner's hub and its own leaves always travel together, an L/R
% mislabel would be cosmetic; nothing geometric depends on it.
M.wheelHub = struct( ...
    'FL', [ 0.8851  1.3000  0.3480], 'FR', [-0.8109  1.3000  0.3480], ...
    'RL', [ 0.8369 -1.5020  0.3660], 'RR', [-0.7625 -1.5020  0.3660]);
% Implied geometry, measured: wheelbase 2.8020, FRONT track 1.6960, REAR track
% 1.5994, tyre radius 0.3468 front / 0.3637 rear, tyre width 0.277 / 0.337.
% Against vp: l 2.800 (+0.07%), t 1.705 (front -0.5%, rear -6.2%), Rw 0.370
% (front -6.3%, rear -1.7%). ASSET DEFECT, recorded not corrected: the whole
% Wheel1A_3D_48 subtree sits 37 mm to +x of the body centreline (both axles: the
% front hubs are +0.8851/-0.8109, the rear +0.8369/-0.7625, mid +0.0371 and
% +0.0372, against a body shell that is symmetric to 5 mm). Draw the wheels at
% their own measured hubs, as above, or symmetrise by subtracting 0.037 from the
% x column -- but do not mix the two.
M.wheelRotorPat  = 'Combined3DWheel';   % ancestor-path match: the spinning part
M.wheelStatorPat = 'Calliper';          % ancestor-path match: the fixed calliper

% --- the Tur's own two flank duct strips (become the actuated front flap) -----
% USER CORRECTION 2026-09-17 (screenshot): the moving element is NOT the central
% honeycomb vent band above (red circle in the screenshot -- that band is left in
% the body untouched now) but the pair of horizontal duct strips on the nose
% FLANK, one per side, between the headlight and the splitter lip (green circle).
% Asset frame, RIGHT-side (+x) box measured; the LEFT-side (-x) row is its exact
% mirror (this body is symmetric to a few mm, same reasoning M.pylonBox already
% relies on for the Agil struts).
%
% HOW MEASURED. Three passes, cross-checked, neither a screen-pixel guess:
%  (a) Marker-box renders from the user's own vantage (camT [4.5 -2.2 0.55],
%      camR [0 6 155]) plus a low front-on view, simulink/work/t1bE*.png /
%      t1b_marker.m -- iterated through candidate boxes (t1bm1/t1bv/t1bA/t1bD/
%      t1bE/t1bElit); the brief's own opening guess (x [0.55 0.95] y [2.05 2.35]
%      z [0.20 0.34]) measures EMPTY (0 face centroids -- it sits in the open
%      diffuser mouth behind the splitter, not on the flank) -- confirmed both
%      by render (floats in the underbody cavity with true-black void around
%      it, simulink/work/t1bm1_uservant_bright_crop.png) and by the
%      face-centroid count.
%  (b) Face-centroid scans (simulink/work/t1b_meas3..5.m, t1b_zbins.m,
%      t1b_colors.m; localExtract itself splits on face centroid, so this is the
%      same rule the builder uses) of the nose-flank leaves -- the broad "empty
%      texture" body-colour leaf (near-black, RGB [0.028 0.028 0.028] --
%      Actor13/Actor39 in a throwaway inspect walk) plus the carbon05_black
%      texture leaf, restricted to the region below the headlight
%      (LightA-textured leaf, z >= 0.466, x [0.382 0.884] y [1.691 2.181],
%      itself a single swept diagonal blade -- the "eye" shape, not the target)
%      and above the splitter (z <= ~0.20 at the outboard corner). The brief's
%      box (y 2.05..2.35) sits at the very nose TIP, where the outboard flank
%      has already tapered inward -- real geometry at x > 0.5 only starts
%      further BACK, y <~ 2.15 (t1b_meas4/5 face-centroid maps).
%  (c) THE DECIDING PASS: the real extraction, rendered deployed and tucked
%      (simulink/work/zc1_nose_dep/tuck.png, zc2_ variants, and
%      zc2_quarter_bright.png, all READ). Passes (a)/(b) alone could not
%      independently confirm a visual match the way they did for the vent
%      (renders of this region come back true-black even where real geometry
%      exists -- RGB [0.028 0.028 0.028] IS the material, not an absent
%      surface), so the box was widened once more from the (a)/(b) estimate
%      (x [0.55 0.88] y [1.80 2.12] z [0.28 0.42], 324 tris/strip) to the
%      shipped box (493 tris/strip) by rendering the actual cut piece, not a
%      translucent marker, and reading its position against the headlight and
%      the splitter blade directly. The shipped box sits right at the front
%      corner of the bumper, below the headlight and above the splitter's
%      visible lip, outboard of the vent (matches the screenshot: green circle
%      left of red in simulink/work/user_screenshot_zoom.png, i.e. closer to
%      the wheel) -- qualitatively the right place, but that first cut's
%      z-span (0.24..0.44, 20 cm) crossed several unrelated creases, so the
%      EXTRACTED SHAPE came out irregular/jagged rather than a crisp "duct
%      strip" silhouette. Narrowed in (d).
%  (d) FIX ROUND 1 (2026-09-18), z NARROWED TO THE DUCT LIP ITSELF. The
%      x and y columns are unchanged -- the position was already right -- and
%      only the z-span moved, from 0.24..0.44 to 0.32..0.38. Measured by
%      simulink/work/t1bf_zscan.m / t1bf_zfine.m, which bin the SAME face
%      centroids the builder splits on and, for each candidate sub-band, also
%      replay localBandStrip's own per-station rule (yE = max y at the
%      station, z0 = min z within one chord of it) to predict how jagged the
%      station band drawn on that band's lower edge would be:
%
%        z-band          tris/strip   z0 range   z0 std   worst station jump
%        0.24..0.44 old      493       0.219 m   0.063 m       0.103 m
%        0.32..0.38 new      330       0.037 m   0.010 m       0.021 m
%
%      The 10 mm z histogram over x [0.50 0.90], y [1.75 2.15] shows ONE
%      continuous dense band -- 37/89/94/71 faces in the four bins from 0.33 to
%      0.37, against 1..12 per bin below 0.33 and 15..23 per bin from 0.38 to
%      0.44. Those sparse neighbours are the creases the old box swept up; the
%      real lip occupies z 0.325..0.379, and 0.32..0.38 takes it with about
%      5 mm of margin each side and nothing else. 330 triangles per strip is
%      well clear of the builder's MIN_TRIS = 100 fallback.
M.flapBoxes = [ 0.50  0.90   1.75  2.15   0.32  0.38     % RIGHT strip (+x)
                -0.90 -0.50   1.75  2.15   0.32  0.38 ]; % LEFT strip (-x), mirror
% Hinge = the strips' UPPER, REARWARD edge line, lateral, x spanning BOTH strips
% at their common y/z, so a single WingFWHinge actor drives both together (the
% flap's lower edge swings forward and down on deploy, flush when tucked).
% z re-measured with the narrowed band (fix round 1): the lip's top at the
% rearward station y 1.75..1.80 is 0.377, so the hinge line sits on the box's
% own 0.38 ceiling, as it did on the old 0.44 one. Asset frame.
M.flapHinge = [-0.90 1.75 0.38
                0.90 1.75 0.38];

% --- the Agil's rear wing and pylons (extracted, then placed on the Tur deck) --
% AGIL frame, same convention. Measured from t0_meas3 maps A2 (top, z>=0.95), A3a
% (side, 0.60<|x|<1.02), A3b (0.22<|x|<0.45), A3c (|x|<0.20) and the A4 pylon
% x-histogram, and confirmed with marker boxes on simulink/work/t0_agil3_wing_side.png
% and _wing_r34.png.
%
% The wing is SWEPT: at the centreline it spans y -2.42..-2.10, at 0.22<|x|<0.45
% y -2.42..-2.00, and at the tips (|x| up to 0.94) y -2.28..-1.86, sitting at
% z 0.96..1.12. The box below takes all of it with a 2 cm margin.
% KNOWN RISK, check it on the first render: the engine-cover hump at |x| < 0.22,
% y -1.94..-1.86 reaches z ~0.97, just inside this box's floor. If a scrap of deck
% skin comes out with the plane, raise z0 from 0.96 to 0.98 -- that costs only the
% wing's lowest 2 cm of lower skin at the tips.
M.wingBox  = [-1.02 1.02 -2.46 -1.86 0.96 1.16];
% PYLONS. Two struts, NOT a centre swan neck: the A4 histogram over
% y[-2.12 -1.78], z[0.88 1.00] has 2150 vertices per side between |x| 0.26 and
% 0.36 and nothing comparable in between, and the pylon top runs z 1.010..1.040
% over y -2.16..-2.00 before dropping to the deck at y -1.98.
% THE x COLUMN IS THE +x STRUT ONLY. The -x strut is its exact mirror; a builder
% must apply this box to BOTH signs of x. A single box spanning the centre cannot
% be used: the engine cover fills |x| < 0.24 at the same y and z.
% PRECEDENCE: this box overlaps M.wingBox where the pylon meets the wing
% (y -2.20..-1.98, z 0.96..1.00). The pylons win there; apply this box first and
% the wing box to what is left.
M.pylonBox = [0.24 0.40 -2.20 -1.98 0.86 1.00];
% Pivot = the pylon-top line, lateral, at the struts' own x centre (0.31, the A4
% histogram peak) and at the station where the tops are highest (y -2.10,
% z 1.040). The delivered wing geometry is station 0; the commanded angle is a
% rotation about this line.
M.wingPivot = [-0.31 -2.10 1.040
                0.31 -2.10 1.040];
% Where the extracted pylons are cut off, i.e. the feet, in the AGIL frame.
M.pylonFeetZ = 0.860;
% Tur deck height under those feet: max z of the Tur's own shells over
% |x| 0.25..0.39, measured per 0.05 m y slice across the foot span -- 0.854 at
% y -2.05, 0.876 at -2.00, 0.874 at -1.95, 0.898 at -1.90..-1.80. The value below
% is the deck at the foot span's own centre.
M.deckZ = 0.876;
% Agil -> Tur rigid offset for the whole wing assembly. Both cars are the same
% base model at the same scale, so x and y are zero by construction (verified: the
% two decks agree to 3..7 mm at the stations clear of both the wing and the
% pylons, y -1.75..-1.65). z is M.deckZ - M.pylonFeetZ, i.e. the lift that stands
% the cut feet on the Tur's deck. Task 1 re-measures the deck under the feet on
% the rewritten shell and is expected to trim this by a few millimetres.
M.wingPlace = [0 0 0.016];

M.band = 0.05;                               % m, station-colour band chord
M.hideNodes = {};

% --- provenance ---------------------------------------------------------------
% Both models: same author, same licence, same "Based on a CSR2 3d model"
% description. See LICENSE.md in this folder and in ../zenvo_agil/.
M.licence = 'CC BY-NC-SA 4.0';
M.author  = 'Ddiaz Design';
M.url = {'https://sketchfab.com/3d-models/2026-zenvo-aurora-tur-a9420f263dac41e99e42112c122987a8', ...
         'https://sketchfab.com/3d-models/2026-zenvo-aurora-agil-7659b0982c9f4550a674bc73e6d3497e'};
M.provenance = 'based on a CSR2 3d model';
end

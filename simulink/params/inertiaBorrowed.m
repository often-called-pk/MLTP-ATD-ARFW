function inrt = inertiaBorrowed()
%INERTIABORROWED Roll / pitch / yaw inertia tensor for the ARFWr RT plant's
%rigid body (Plant.slx "Body 6DOF", mask parameter Iveh).
%
%   inrt = inertiaBorrowed()
%
% Returns a struct: Ixx, Iyy, Izz (kg.m^2, principal, vehicle-fixed axes),
% Iveh (the 3x3 diagonal tensor), source (one-line provenance string),
% and check (the plausibility cross-check computed below).
%
% =====================================================================
%  READ THIS FIRST -- THE FILE NAME IS HISTORICAL, THE NUMBERS ARE NOT
%  BORROWED
% =====================================================================
% The plant build put PLACEHOLDERS in Plant.slx's model workspace:
%     phIxx = (430/2100)*vp.I_z = 593.8      phIyy = (1900/2100)*vp.I_z = 2623.8
% scaled off the R2025a Vehicle Body 6DOF block's OWN default inertia-tensor
% ratios, on the stated grounds that there is "no supplier data" and "No
% Ixx/Iyy anywhere in the repo" (build_plant.m:65-72). A follow-up pass was
% briefed to replace them with BORROWED values by the radius-of-gyration
% method, hence this file's name.
%
% That premise turned out to be WRONG. Zenvo's own vehicle-parameter workbook,
% already in this checkout at
%     reference/vehicle/Zenvo_PublicAcademicProjects_VehicleModelParameters.xlsx
% sheet "Vehicle", block "Masses and inertiae" -> "Vehicle Inertia", rows 32-34
% (version date 31-Jan-2026), publishes ALL THREE:
%     Ixx (Roll)  =  500 kg.m^2
%     Iyy (Pitch) = 2500 kg.m^2
%     Izz (Yaw)   = 2900 kg.m^2
% Izz is the number Parameters/vehParams.m:46 already carries as vp.I_z ("Izz
% from sheet"), so the same sheet, the same row block and the same version date
% are the established provenance for the one inertia this repo does use. Only
% Ixx and Iyy were never transcribed.
%
% This function therefore ships the SUPPLIER values (tagged % ZENVO below), not
% borrowed ones, and keeps the borrowed radius-of-gyration derivation the task
% asked for as an executable PLAUSIBILITY CROSS-CHECK (tagged % BORROWED) that
% is asserted against them at load time. The file name is kept because the task
% brief, the plan and the commit pathspec all name it.
%
% OPEN ITEM for a later task (deliberately NOT done here -- it changes the
% offline solver's parameter set, which is out of this task's scope):
% Parameters/vehParams.m should carry vp.I_x / vp.I_y from the same sheet rows,
% next to vp.I_z. Until it does, this file is the only place they exist.
%
% =====================================================================
%  CROSS-CHECK 1 (BORROWED) -- radius of gyration, calibrated solid box
% =====================================================================
% Method: idealise the vehicle as a homogeneous rectangular prism of the
% vehicle's own total mass, of width = front track and height = 2*hcg (a box
% centred on the CG height), and SOLVE its length from the supplier's Izz.
% The same box then predicts Ixx and Iyy with no further free parameters:
%
%     Izz = m*(Lx^2 + Ly^2)/12   ->   Lx = sqrt(12*Izz/m - Ly^2)
%     Ixx = m*(Ly^2 + Lz^2)/12         Iyy = m*(Lx^2 + Lz^2)/12
%
% (standard rigid-body result for a homogeneous rectangular prism about its
% centroidal axes -- any classical-mechanics text; e.g. Meriam & Kraige,
% Engineering Mechanics: Dynamics, Appendix B.)
%
% This is the radius-of-gyration method the brief asked for, written in its
% equivalent box form: k_x^2 = (Ly^2+Lz^2)/12 etc. It is anchored on the
% vehicle's OWN measured yaw inertia rather than on a table value read out of a
% textbook, which is the more defensible choice here precisely because the
% anchor exists.
%
% =====================================================================
%  CROSS-CHECK 2 (BORROWED) -- non-dimensional radii and the dynamic index
% =====================================================================
% Vehicle inertia is conventionally characterised by non-dimensional radii of
% gyration, k_x/track and k_y/wheelbase, and by the DYNAMIC INDEX
%     DI = k_z^2/(a*b)
% (a, b = CG-to-front-axle and CG-to-rear-axle distances). DI = 1 decouples the
% front and rear ride modes and is the classical design target; most road cars
% sit near it. The dynamic index and the practice of quoting non-dimensional
% radii of gyration are standard vehicle-dynamics material:
%
%   Milliken, W.F. and Milliken, D.L., "Race Car Vehicle Dynamics",
%   SAE International, Warrendale PA, 1995 (the RCVD-class source the brief
%   asked for) -- and Gillespie, T.D., "Fundamentals of Vehicle Dynamics",
%   SAE International, 1992, ride chapter, for the dynamic index.
%
% HONESTY NOTE: neither book is in this checkout, and no copy was read while
% writing this file. They are cited for the METHOD and for the DI ~ 1 rule of
% thumb -- both textbook-standard -- and NOT for any numeric table entry. No
% number below is taken from either source. The acceptance bands in the asserts
% are deliberately wide engineering-plausibility bands, chosen by me, and are
% labelled as such.
%
% =====================================================================

%% ---- SUPPLIER VALUES ------------------------------------------------
% ZENVO(VehicleModelParameters.xlsx, sheet "Vehicle", "Vehicle Inertia",
% rows 32-34, ver. 31-Jan-2026). Principal, vehicle-fixed axes; the sheet
% gives no products of inertia, so the tensor is taken diagonal.
inrt.Ixx = 500;     % ZENVO: Ixx (Roll)  [kg.m^2]
inrt.Iyy = 2500;    % ZENVO: Iyy (Pitch) [kg.m^2]
inrt.Izz = 2900;    % ZENVO: Izz (Yaw)   [kg.m^2]  == Parameters/vehParams.m:46 vp.I_z

inrt.Iveh = diag([inrt.Ixx, inrt.Iyy, inrt.Izz]);
inrt.source = ['Zenvo_PublicAcademicProjects_VehicleModelParameters.xlsx, ' ...
               'sheet Vehicle, "Vehicle Inertia" rows 32-34, ver. 31-Jan-2026'];

%% ---- frozen literal inputs for the cross-checks ---------------------
% Hand-copied from Parameters/vehParams.m as it stood on 2026-09-02, cited by
% line number -- the same convention simulink/params/suspensionBorrowed.m uses,
% so this file never runs vehParams.m and never depends on load order.
m    = 1602;    % BORROWED(vehParams.m:45): total vehicle mass  (kg)
t_f  = 1.74;    % BORROWED(vehParams.m:49): track width         (m)
L    = 2.8;     % BORROWED(vehParams.m:50): wheelbase           (m)
l_f  = 1.596;   % BORROWED(vehParams.m:55): CoG -> front axle   (m)
l_r  = 1.204;   % BORROWED(vehParams.m:56): CoG -> rear  axle   (m)
hcg  = 0.44;    % BORROWED(vehParams.m:58): CG height           (m)

%% ---- CROSS-CHECK 1: calibrated homogeneous box ----------------------
Ly = t_f;                                   % box width  = front track
Lz = 2*hcg;                                 % box height = 2*CG height
Lx = sqrt(12*inrt.Izz/m - Ly^2);            % box length, SOLVED from Izz
IxxBox = m*(Ly^2 + Lz^2)/12;
IyyBox = m*(Lx^2 + Lz^2)/12;

inrt.check.boxLx     = Lx;
inrt.check.IxxBox    = IxxBox;
inrt.check.IyyBox    = IyyBox;
inrt.check.IxxRelDev = (IxxBox - inrt.Ixx)/inrt.Ixx;
inrt.check.IyyRelDev = (IyyBox - inrt.Iyy)/inrt.Iyy;

% BORROWED acceptance band, my choice: a single-parameter box idealisation of a
% real car is doing well to land inside 25 %. Measured 2026-09-02: +1.5 % on
% Ixx, +4.0 % on Iyy, with a solved box length of 4.32 m against the sheet's
% published body length of 4.819 m (row 2) -- i.e. the box is slightly shorter
% than the car, exactly as it must be, because a real car carries its mass
% inboard of its bumpers.
assert(abs(inrt.check.IxxRelDev) < 0.25 && abs(inrt.check.IyyRelDev) < 0.25, ...
    'inertiaBorrowed:boxCheck', ...
    ['calibrated-box cross-check disagrees with the supplier tensor by more ' ...
     'than 25%% (Ixx %+.1f%%, Iyy %+.1f%%) -- one of the two is wrong, do not ' ...
     'silently widen this band'], 100*inrt.check.IxxRelDev, 100*inrt.check.IyyRelDev);

%% ---- CROSS-CHECK 2: non-dimensional radii + dynamic index -----------
inrt.check.kx        = sqrt(inrt.Ixx/m);
inrt.check.ky        = sqrt(inrt.Iyy/m);
inrt.check.kz        = sqrt(inrt.Izz/m);
inrt.check.kxOverT   = inrt.check.kx/t_f;
inrt.check.kyOverL   = inrt.check.ky/L;
inrt.check.dynIndex  = inrt.check.kz^2/(l_f*l_r);

% BORROWED acceptance bands, my choice, as engineering plausibility gates only:
%   k_x/track      0.20 .. 0.50   (measured 0.321)
%   k_y/wheelbase  0.30 .. 0.60   (measured 0.446)
%   dynamic index  0.70 .. 1.30   (measured 0.942 -- close to the DI = 1 target)
assert(inrt.check.kxOverT > 0.20 && inrt.check.kxOverT < 0.50, ...
    'inertiaBorrowed:kx', 'k_x/track = %.3f is outside the plausibility band [0.20 0.50]', ...
    inrt.check.kxOverT);
assert(inrt.check.kyOverL > 0.30 && inrt.check.kyOverL < 0.60, ...
    'inertiaBorrowed:ky', 'k_y/wheelbase = %.3f is outside the plausibility band [0.30 0.60]', ...
    inrt.check.kyOverL);
assert(inrt.check.dynIndex > 0.70 && inrt.check.dynIndex < 1.30, ...
    'inertiaBorrowed:DI', 'dynamic index = %.3f is outside the plausibility band [0.70 1.30]', ...
    inrt.check.dynIndex);

inrt.notes = sprintf([ ...
    'Ixx/Iyy/Izz = %g/%g/%g kg.m^2, SUPPLIER data (%s). Cross-checks: ' ...
    'calibrated homogeneous box (Lx solved from Izz) predicts Ixx %+.1f%% / ' ...
    'Iyy %+.1f%%; k_x/track = %.3f, k_y/wheelbase = %.3f, dynamic index = %.3f.'], ...
    inrt.Ixx, inrt.Iyy, inrt.Izz, inrt.source, ...
    100*inrt.check.IxxRelDev, 100*inrt.check.IyyRelDev, ...
    inrt.check.kxOverT, inrt.check.kyOverL, inrt.check.dynIndex);
end

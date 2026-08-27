% plotSDI.m - plot results in Simulink's Simulation Data Inspector (SDI).
%
% Signals cover the states, inputs, auxiliary variables, optimised parameters,
% derived vehicle quantities (slips, tyre forces), selected path constraints for
% debugging, and the track. Extra 'custom' signals can be calculated and logged
% in the Postprocessing section of MLTP.m. Retrieve any of them by identifier:
%
%   STATES        vx  vy  r  n  eps
%                 (longitudinal/lateral velocity, yaw rate, normal distance to
%                  the centreline, angle relative to the centreline tangent)
%   INPUTS        delta (road-wheel steer), Tdrive, Tbrake (total at the wheels)
%   AUX           loadTransferX  loadTransferY
%   VEHICLE       sa_f sa_r   slip angles          fx_f fx_r   longitudinal force
%                 sx_f sx_r   slip ratios          fy_f fy_r   lateral force
%                 f_drag      aero drag            fz_f fz_r   vertical force
%                 f_lift      aero downforce       (tyre forces are local coords)
%   PARAMETERS    wB (weight distribution)  brkB (brake balance)
%   TRACK         s  k  x  y
%   CONSTRAINTS   BrTh      brake/throttle overlap
%                 ltx_eq    longitudinal load transfer
%                 lty_eq    lateral load transfer
%                 gu_*      per-tyre grip utilisation (dimensionless)
%                 fz_lim_*  per-tyre vertical load limit margin
%
% An error can occur if SDI already holds plots in a visualization style other
% than 'time plot'.


%% Plot in SDI

%-Clear previously plotted signals
Simulink.sdi.clearAllSubPlots;

%-Set plot layotu (rows, columns)
Simulink.sdi.setSubPlotLayout(3,4);

%-Select signals to plot

% 1x1 - velocity
vxSig = getSignalsByName(data.sdi.run,'vx'); vxSig.plotOnSubPlot(1,1,true);

% 2x1 - steering angle
deltaSig = getSignalsByName(data.sdi.run,'delta'); deltaSig.plotOnSubPlot(2,1,true);

% 3x1 - Torque
fmotor_fSig = getSignalsByName(data.sdi.run,'T_motor'); fmotor_fSig.plotOnSubPlot(3,1,true);
fbrakeSig = getSignalsByName(data.sdi.run,'T_brake');   fbrakeSig.plotOnSubPlot(3,1,true);

% 1x2 - grip utilisation (dimensionless, 1 = on the friction-ellipse limit)
guflSig = getSignalsByName(data.sdi.run,'gu_fl'); guflSig.plotOnSubPlot(1,2,true);
gufrSig = getSignalsByName(data.sdi.run,'gu_fr'); gufrSig.plotOnSubPlot(1,2,true);
gurlSig = getSignalsByName(data.sdi.run,'gu_rl'); gurlSig.plotOnSubPlot(1,2,true);
gurrSig = getSignalsByName(data.sdi.run,'gu_rr'); gurrSig.plotOnSubPlot(1,2,true);

% 2x2 - torque distribution inputs
if pt.ATD == 0
    Pmotorsig = getSignalsByName(data.sdi.run,'P_motor (kW)'); Pmotorsig.plotOnSubPlot(2,2,true);
    fmotor_fSig.plotOnSubPlot(2,2,true);
elseif pt.ATD == 1
    ATD_FLSig = getSignalsByName(data.sdi.run,'ATD_FL'); ATD_FLSig.plotOnSubPlot(2,2,true);
    ATD_FRSig = getSignalsByName(data.sdi.run,'ATD_FR'); ATD_FRSig.plotOnSubPlot(2,2,true);
    ATD_RLSig = getSignalsByName(data.sdi.run,'ATD_RL'); ATD_RLSig.plotOnSubPlot(2,2,true);
    ATD_RRSig = getSignalsByName(data.sdi.run,'ATD_RR'); ATD_RRSig.plotOnSubPlot(2,2,true);
end

% 3x2 - torque per wheel
TflSig = getSignalsByName(data.sdi.run,'T_fl'); TflSig.plotOnSubPlot(3,2,true);
TfrSig = getSignalsByName(data.sdi.run,'T_fr'); TfrSig.plotOnSubPlot(3,2,true);
TrlSig = getSignalsByName(data.sdi.run,'T_rl'); TrlSig.plotOnSubPlot(3,2,true);
TrrSig = getSignalsByName(data.sdi.run,'T_rr'); TrrSig.plotOnSubPlot(3,2,true);

% 1x3 - load transfer
ltxSig = getSignalsByName(data.sdi.run,'loadTransferX'); ltxSig.plotOnSubPlot(1,3,true);
ltySig = getSignalsByName(data.sdi.run,'loadTransferY'); ltySig.plotOnSubPlot(1,3,true);

% 2x3 - aerodynamics
if vp.ActAero == 0
    f_liftSig = getSignalsByName(data.sdi.run,'f_lift'); f_liftSig.plotOnSubPlot(2,3,true);
    f_dragSig = getSignalsByName(data.sdi.run,'f_drag'); f_dragSig.plotOnSubPlot(2,3,true);
elseif vp.ActAero == 1
    % f_lift/f_drag are config-independent post-processing outputs (MLTP.m always
    % logs them) - ActiveRW needs them on this subplot just as much as Static does,
    % it is the wing angle vs the forces it produces that makes the panel readable.
    f_liftSig = getSignalsByName(data.sdi.run,'f_lift'); f_liftSig.plotOnSubPlot(2,3,true);
    f_dragSig = getSignalsByName(data.sdi.run,'f_drag'); f_dragSig.plotOnSubPlot(2,3,true);
    activeAeroRWSig = getSignalsByName(data.sdi.run,'activeAeroRW'); activeAeroRWSig.plotOnSubPlot(2,3,true);
    if any(vp.rwMandate == [6 7])
        % ARFWd/ARFWr: second wing surface (FW flap, row nu-2)
        activeAeroFWSig = getSignalsByName(data.sdi.run,'activeAeroFW'); activeAeroFWSig.plotOnSubPlot(2,3,true);
    end
elseif vp.ActAero == 2
    activeAeroFWSig = getSignalsByName(data.sdi.run,'activeAeroFW'); activeAeroFWSig.plotOnSubPlot(2,3,true);  
    activeAeroRWSig = getSignalsByName(data.sdi.run,'activeAeroRW'); activeAeroRWSig.plotOnSubPlot(2,3,true);  
elseif vp.ActAero == 3
    activeAeroFLSig = getSignalsByName(data.sdi.run,'activeAeroFL'); activeAeroFLSig.plotOnSubPlot(2,3,true);
    activeAeroFRSig = getSignalsByName(data.sdi.run,'activeAeroFR'); activeAeroFRSig.plotOnSubPlot(2,3,true);
    activeAeroRWSig = getSignalsByName(data.sdi.run,'activeAeroRW'); activeAeroRWSig.plotOnSubPlot(2,3,true);
    activeAeroTWSig = getSignalsByName(data.sdi.run,'activeAeroTW'); activeAeroTWSig.plotOnSubPlot(2,3,true);              
end

% 3x3 - aerodynamic forces
f_lift_flSig = getSignalsByName(data.sdi.run,'f_lift_fl'); f_lift_flSig.plotOnSubPlot(3,3,true);
f_lift_frSig = getSignalsByName(data.sdi.run,'f_lift_fr'); f_lift_frSig.plotOnSubPlot(3,3,true);
f_lift_rlSig = getSignalsByName(data.sdi.run,'f_lift_rl'); f_lift_rlSig.plotOnSubPlot(3,3,true);
f_lift_rrSig = getSignalsByName(data.sdi.run,'f_lift_rr'); f_lift_rrSig.plotOnSubPlot(3,3,true);
f_sideSig = getSignalsByName(data.sdi.run,'f_side'); f_sideSig.plotOnSubPlot(3,3,true);

% 1x4 - longitudinal tyre forces
fxflSig = getSignalsByName(data.sdi.run,'fx_fl'); fxflSig.plotOnSubPlot(1,4,true);
fxfrSig = getSignalsByName(data.sdi.run,'fx_fr'); fxfrSig.plotOnSubPlot(1,4,true);
fxrlSig = getSignalsByName(data.sdi.run,'fx_rl'); fxrlSig.plotOnSubPlot(1,4,true);
fxrrSig = getSignalsByName(data.sdi.run,'fx_rr'); fxrrSig.plotOnSubPlot(1,4,true);

% 2x4 - lateral tyre forces
fyflSig = getSignalsByName(data.sdi.run,'fy_fl'); fyflSig.plotOnSubPlot(2,4,true);
fyfrSig = getSignalsByName(data.sdi.run,'fy_fr'); fyfrSig.plotOnSubPlot(2,4,true);
fyrlSig = getSignalsByName(data.sdi.run,'fy_rl'); fyrlSig.plotOnSubPlot(2,4,true);
fyrrSig = getSignalsByName(data.sdi.run,'fy_rr'); fyrrSig.plotOnSubPlot(2,4,true);

% 3x4 - vertical tyre forces
fzflSig = getSignalsByName(data.sdi.run,'fz_fl'); fzflSig.plotOnSubPlot(3,4,true);
fzfrSig = getSignalsByName(data.sdi.run,'fz_fr'); fzfrSig.plotOnSubPlot(3,4,true);
fzrlSig = getSignalsByName(data.sdi.run,'fz_rl'); fzrlSig.plotOnSubPlot(3,4,true);
fzrrSig = getSignalsByName(data.sdi.run,'fz_rr'); fzrrSig.plotOnSubPlot(3,4,true);

%OTHER
% wheel speed
OmflSig = getSignalsByName(data.sdi.run,'Om_fl'); %OmflSig.plotOnSubPlot(1,2,true);
OmfrSig = getSignalsByName(data.sdi.run,'Om_fr'); %OmfrSig.plotOnSubPlot(1,2,true);
OmrlSig = getSignalsByName(data.sdi.run,'Om_rl'); %OmrlSig.plotOnSubPlot(1,2,true);
OmrrSig = getSignalsByName(data.sdi.run,'Om_rr'); %OmrrSig.plotOnSubPlot(1,2,true);
kSig = getSignalsByName(data.sdi.run,'k'); % kSig.plotOnSubPlot(1,3,true);

%-vehicle
%%-tyres
sxflSig = getSignalsByName(data.sdi.run,'sx_fl'); %sxflSig.plotOnSubPlot(3,2,true);
sxfrSig = getSignalsByName(data.sdi.run,'sx_fr'); %sxfrSig.plotOnSubPlot(3,2,true);
sxrlSig = getSignalsByName(data.sdi.run,'sx_rl'); %sxrlSig.plotOnSubPlot(3,2,true);
sxrrSig = getSignalsByName(data.sdi.run,'sx_rr'); %sxrrSig.plotOnSubPlot(3,2,true);
saflSig = getSignalsByName(data.sdi.run,'sa_fl'); %saflSig.plotOnSubPlot(2,2,true);
safrSig = getSignalsByName(data.sdi.run,'sa_fr'); %safrSig.plotOnSubPlot(2,2,true);
sarlSig = getSignalsByName(data.sdi.run,'sa_rl'); %sarlSig.plotOnSubPlot(2,2,true);
sarrSig = getSignalsByName(data.sdi.run,'sa_rr'); %sarrSig.plotOnSubPlot(2,2,true);



%%-Energy
Emotorsig = getSignalsByName(data.sdi.run,'E_motor (kWh)'); %Emotorsig.plotOnSubPlot(4,1,true);

%%-aerodynamic forces
f_side_flSig = getSignalsByName(data.sdi.run,'f_side_fl'); %f_side_flSig.plotOnSubPlot(3,2,true);
f_side_frSig = getSignalsByName(data.sdi.run,'f_side_fr'); %f_side_frSig.plotOnSubPlot(3,2,true);
f_side_rlSig = getSignalsByName(data.sdi.run,'f_side_rl'); %f_side_rlSig.plotOnSubPlot(3,2,true);
f_side_rrSig = getSignalsByName(data.sdi.run,'f_side_rr'); %f_side_rrSig.plotOnSubPlot(3,2,true);
% Mx_rcSig = getSignalsByName(data.sdi.run,'Mx_rc'); Mx_rcSig.plotOnSubPlot(1,3,true);
% My_flSig = getSignalsByName(data.sdi.run,'My_fl'); %My_flSig.plotOnSubPlot(3,2,true);

%%-e-motor speed
OmMotorSig = getSignalsByName(data.sdi.run,'Om_motor'); %OmMotorSig.plotOnSubPlot(4,2,true);

% Open SDI (Simulation Data Inspector)
Simulink.sdi.view


function law = ecuLawParams()
% BCN fitted (Validation/fitReactiveLaw.m 2026-08-11); TbMax=7000 Nm (vehParams:288 w1frac basis)
law = struct('k1',0.000385,'k2',0.022565,'w1',0.003697,'w2',0.005545, ...
    'stRW',[-10 0 10 15],'stFW',[-25 -20 0],'TbMax',7000, ...
    'w1frac',0.04, ...            % brake tanh onset frac of TbMax
    'hystFrac',0.15,'dwellMin',0.2,'Ts',0.01);   % snap: hysteresis frac of gate width, min dwell s, ECU step
end

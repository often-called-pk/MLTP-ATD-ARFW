function [fx, fy, mux, muy] = tyreMFnum(ty, Fz0eff, fz, sx, sa)
% NUMERIC TWIN of tyreMF() in Scripts/vehModel.m - KEEP THE FORMULAS IDENTICAL.
% Vectorised (elementwise) evaluation of the per-axle Pacejka MF5.2 model:
% pure-slip sine curves weighted by the combined-slip cosine functions
% Gxa/Gyk. All MF shift and vertical-shift parameters are zero in the
% supplier data, so the weighting denominators G(SH)=1 and SVyk=0 drop out.
% ty = vp.tyre_f or vp.tyre_r; Fz0eff = (possibly shifted) nominal load.
dfz = (fz - Fz0eff)./Fz0eff;
mux = ty.pDx1 + ty.pDx2.*dfz;
muy = ty.pDy1 + ty.pDy2.*dfz;
% pure slip
fx0 = mux.*fz.*sin(ty.Cx.*atan(ty.Bx.*sx - ty.Ex.*(ty.Bx.*sx - atan(ty.Bx.*sx))));
fy0 = muy.*fz.*sin(ty.Cy.*atan(ty.By.*sa - ty.Ey.*(ty.By.*sa - atan(ty.By.*sa))));
% combined-slip weighting
Bxa = ty.rBx1.*cos(atan(ty.rBx2.*sx));
Gxa = cos(ty.rCx1.*atan(Bxa.*sa - ty.rEx1.*(Bxa.*sa - atan(Bxa.*sa))));
Byk = ty.rBy1.*cos(atan(ty.rBy2.*sa));
Gyk = cos(ty.rCy1.*atan(Byk.*sx - ty.rEy1.*(Byk.*sx - atan(Byk.*sx))));
fx = Gxa.*fx0;
fy = Gyk.*fy0;
end

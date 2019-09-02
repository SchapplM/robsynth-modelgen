% Regressormatrix des Vektors der Gravitationskräfte der PKM-Plattform (für 6FG)
% Bezogen auf Inertialparameter
% 
% Eingabe:
% phi [3x1]
%   XYZ-Euler-Winkel des Plattform-KS
% g [3x1]
%   Gravitationsvektor im Basis-KS
% 
% Ausgabe:
% taug_reg [6x10]
%   Regressor der Gravitationskräfte (Kraft und Moment) im Basis-KS
%   Reihenfolge der Inertialparameter: XX, XY, XZ, YY, YZ, ZZ, MX, MY, MZ, M

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function taug_reg = rigidbody_pkm_pf_gravload_reg2(phi, g)

%% Initialisierung
x_all = [zeros(3,1); phi];

g1 = g(1);
g2 = g(2);
g3 = g(3);

%% Berechnung
% Symbolische Berechnung in robot_para_plattform_rotmat_dynamics_regressor.mw
% Aus invdyn_floatb_twist_gplatform_matlab.m
t308 = x_all(6);
t302 = sin(t308);
t309 = x_all(5);
t306 = cos(t309);
t318 = t302 * t306;
t310 = x_all(4);
t304 = sin(t310);
t317 = t304 * t302;
t305 = cos(t308);
t316 = t304 * t305;
t315 = t304 * t306;
t314 = t305 * t306;
t307 = cos(t310);
t313 = t306 * t307;
t312 = t307 * t302;
t311 = t307 * t305;
t303 = sin(t309);
t300 = -t303 * t311 + t317;
t299 = t303 * t312 + t316;
t298 = t303 * t316 + t312;
t297 = -t303 * t317 + t311;
t294 = t303 * g1 - g2 * t315 + g3 * t313;
t293 = g1 * t314 + t298 * g2 + t300 * g3;
t292 = g1 * t318 - t297 * g2 - t299 * g3;
t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, -t303 * t294 + (-t292 * t302 - t293 * t305) * t306; 0, 0, 0, 0, 0, 0, 0, 0, 0, t297 * t292 - t298 * t293 + t294 * t315; 0, 0, 0, 0, 0, 0, 0, 0, 0, t299 * t292 - t300 * t293 - t294 * t313; 0, 0, 0, 0, 0, 0, t303 * t292 - t294 * t318, t303 * t293 - t294 * t314, (-t292 * t305 + t293 * t302) * t306, 0; 0, 0, 0, 0, 0, 0, -t292 * t315 + t297 * t294, -t293 * t315 - t298 * t294, -t298 * t292 - t297 * t293, 0; 0, 0, 0, 0, 0, 0, t292 * t313 + t299 * t294, t293 * t313 - t300 * t294, -t300 * t292 - t299 * t293, 0;];
taug_reg = t1;
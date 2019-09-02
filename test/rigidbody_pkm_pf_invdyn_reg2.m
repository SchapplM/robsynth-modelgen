% Regressormatrix des Vektors der Dynamikkräfte der PKM-Plattform (für 6FG)
% Bezogen auf Inertialparameter
% 
% Eingabe:
% phi [3x1]
%   XYZ-Euler-Winkel des Plattform-KS
% xD [6x1]
%   Ableitung der Plattformkoordinaten (Geschwindigkeit und
%   Euler-Winkel-Ableitung)
% xDD [6x1]
%   2. Ableitung der Plattformkoordinaten (Beschleunigung und
%   2. Ableitung der Euler-Winkel)
% g [3x1]
%   Gravitationsvektor im Basis-KS
% 
% Ausgabe:
% tau_reg [6x10]
%   Regressor der Invers-Dynamik-Kräfte (Kraft und Moment) im Basis-KS
%   Reihenfolge der Inertialparameter: XX, XY, XZ, YY, YZ, ZZ, MX, MY, MZ, M

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tau_reg = rigidbody_pkm_pf_invdyn_reg2(phi, xD, xDD, g)
%% Initialisierung
x_all = [zeros(3,1); phi];
Dx_all = xD;
DDx_all = xDD;

g1 = g(1);
g2 = g(2);
g3 = g(3);

%% Berechnung
% Symbolische Berechnung in robot_para_plattform_rotmat_dynamics_regressor.mw
% Aus invdyn_floatb_twist_platform_matlab.m
t66 = x_all(5);
t58 = cos(t66);
t62 = DDx_all(4);
t63 = Dx_all(6);
t86 = Dx_all(5);
t90 = t58 * t62 + t63 * t86;
t65 = x_all(6);
t54 = sin(t65);
t55 = sin(t66);
t57 = cos(t65);
t61 = DDx_all(5);
t64 = Dx_all(4);
t73 = t54 * t86;
t78 = t58 * t63;
t19 = -t90 * t54 + (t55 * t73 - t57 * t78) * t64 + t57 * t61;
t77 = t58 * t64;
t43 = t57 * t77 + t73;
t46 = t55 * t64 + t63;
t27 = t43 * t46;
t2 = t19 - t27;
t72 = t57 * t86;
t41 = t54 * t77 - t72;
t24 = t41 * t43;
t40 = t55 * t62 + t86 * t77 + DDx_all(6);
t89 = -t24 + t40;
t88 = t27 + t19;
t20 = (-t54 * t78 - t55 * t72) * t64 + t54 * t61 + t90 * t57;
t28 = t41 * t46;
t8 = -t28 - t20;
t87 = -t28 + t20;
t16 = t40 + t24;
t23 = t41 ^ 2;
t25 = t43 ^ 2;
t35 = t46 ^ 2;
t85 = t54 * t58;
t67 = x_all(4);
t56 = sin(t67);
t84 = t56 * t54;
t83 = t56 * t57;
t82 = t56 * t58;
t81 = t57 * t58;
t59 = cos(t67);
t80 = t58 * t59;
t76 = t59 * t54;
t75 = t59 * t57;
t70 = DDx_all(2) - g2;
t69 = DDx_all(3) - g3;
t53 = DDx_all(1) - g1;
t39 = -t55 * t75 + t84;
t38 = t55 * t76 + t83;
t37 = t55 * t83 + t76;
t36 = -t55 * t84 + t75;
t26 = t55 * t53 + t69 * t80 - t70 * t82;
t22 = -t35 + t23;
t21 = -t25 + t35;
t18 = -t23 + t25;
t13 = -t35 - t25;
t12 = -t35 - t23;
t11 = t37 * t70 + t39 * t69 + t53 * t81;
t10 = -t36 * t70 - t38 * t69 + t53 * t85;
t9 = -t23 - t25;
t1 = [0, 0, 0, 0, 0, 0, -t55 * t2 + (t12 * t57 - t54 * t89) * t58, t55 * t87 + (-t13 * t54 - t16 * t57) * t58, t55 * t9 + (-t54 * t8 + t57 * t88) * t58, t55 * t26 + (t10 * t54 + t11 * t57) * t58; 0, 0, 0, 0, 0, 0, t37 * t12 + t2 * t82 + t36 * t89, t36 * t13 - t16 * t37 - t82 * t87, t36 * t8 + t37 * t88 - t9 * t82, -t36 * t10 + t37 * t11 - t26 * t82; 0, 0, 0, 0, 0, 0, t39 * t12 - t2 * t80 + t38 * t89, t38 * t13 - t16 * t39 + t80 * t87, t38 * t8 + t39 * t88 + t9 * t80, -t38 * t10 + t39 * t11 + t26 * t80; t20 * t81 + (t41 * t55 - t46 * t85) * t43, t55 * t18 + (t2 * t57 - t54 * t87) * t58, -t55 * t8 + (-t21 * t54 + t57 * t89) * t58, -t19 * t85 + (-t43 * t55 + t46 * t81) * t41, t55 * t88 + (-t16 * t54 + t22 * t57) * t58, t55 * t40 + (-t41 * t57 + t43 * t54) * t58 * t46, -t55 * t10 + t26 * t85, -t55 * t11 + t26 * t81, (t10 * t57 - t11 * t54) * t58, 0; t37 * t20 + (t36 * t46 - t41 * t82) * t43, -t18 * t82 + t37 * t2 + t36 * t87, t36 * t21 + t37 * t89 + t8 * t82, t36 * t19 + (t37 * t46 + t43 * t82) * t41, t36 * t16 + t37 * t22 - t82 * t88, -t40 * t82 + (-t36 * t43 - t37 * t41) * t46, t10 * t82 - t36 * t26, t11 * t82 + t37 * t26, t37 * t10 + t36 * t11, 0; t39 * t20 + (t38 * t46 + t41 * t80) * t43, t18 * t80 + t39 * t2 + t38 * t87, t38 * t21 + t39 * t89 - t8 * t80, t38 * t19 + (t39 * t46 - t43 * t80) * t41, t38 * t16 + t39 * t22 + t80 * t88, t40 * t80 + (-t38 * t43 - t39 * t41) * t46, -t10 * t80 - t38 * t26, -t11 * t80 + t39 * t26, t39 * t10 + t38 * t11, 0;];
tau_reg = t1;
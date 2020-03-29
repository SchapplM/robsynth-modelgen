% Massenmatrix der PKM-Plattform (für 6FG)
% 
% Eingabe:
% phi [3x1]
%   XYZ-Euler-Winkel des Plattform-KS
% m_num [1x1], rSges_num_mdh [1x3], Icges_num_mdh [1x6]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Ausgabe:
% M [6x6]
%   Massenmatrix (Bezogen auf Winkelgeschwindigkeit im Basis-KS und Moment
%   im Basis-KS)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function M = rigidbody_pkm_pf_inertia_vp1(phi, m_num, rSges_num_mdh, Icges_num_mdh)

%% Initialisierung
x_all = [zeros(3,1); phi];

mE = m_num;

r_sP = rSges_num_mdh;

XX = Icges_num_mdh(1,1);
XY = Icges_num_mdh(1,4);
XZ = Icges_num_mdh(1,5);
YY = Icges_num_mdh(1,2);
YZ = Icges_num_mdh(1,6);
ZZ = Icges_num_mdh(1,3);

%% Berechnung
% Siehe robot_para_plattform_rotmat_dynamics.mw
% Aus inertia_platform_eulxyz_matlab.m
t67 = r_sP(3);
t61 = t67 ^ 2;
t68 = r_sP(2);
t62 = t68 ^ 2;
t101 = (-t61 + t62) * mE + ZZ - YY;
t65 = x_all(5);
t56 = sin(t65);
t59 = cos(t65);
t84 = t56 * t59;
t109 = -0.2e1 * t84;
t53 = t59 ^ 2;
t46 = mE * t67 * t68 - YZ;
t64 = x_all(6);
t55 = sin(t64);
t89 = t55 * t46;
t102 = t101 * t53 + t89 * t109;
t51 = t53 - 0.2e1;
t69 = r_sP(1);
t93 = mE * t69;
t47 = t67 * t93 - XZ;
t75 = t47 * t84;
t48 = -t68 * t93 + XY;
t90 = t48 * t55;
t58 = cos(t64);
t52 = t58 ^ 2;
t63 = t69 ^ 2;
t99 = (-t62 + t63) * mE - XX + YY;
t91 = t99 * t52;
t96 = 0.2e1 * t58;
t106 = (t51 * t90 + t75) * t96 + t51 * t91 + t99 + t102;
t103 = (-t55 * t56 * t99 + t46 * t59) * t58;
t88 = t55 * t47;
t38 = t59 * t88;
t77 = 0.2e1 * t48 * t52;
t86 = t56 * t48;
t24 = t56 * t77 + t103 + t38 - t86;
t66 = x_all(4);
t60 = cos(t66);
t54 = t60 ^ 2;
t57 = sin(t66);
t81 = t57 * t60;
t110 = t106 * t54 - 0.2e1 * t24 * t81 + t91;
t40 = t55 * t69 + t58 * t68;
t72 = t55 * t68 - t58 * t69;
t98 = t72 * t56 + t59 * t67;
t108 = mE * (-t60 * t40 + t98 * t57);
t104 = (-t56 * t67 + t72 * t59) * mE;
t87 = t55 * t58;
t85 = t56 * t57;
t83 = t56 * t60;
t78 = t60 * t48;
t76 = t48 * t87;
t35 = t47 * t58 - t89;
t30 = -t101 - 0.2e1 * t76 - t91;
t27 = (t57 * t40 + t60 * t98) * mE;
t22 = (-t30 * t83 + (-t87 * t99 - t48 + t77) * t57) * t59 - (t46 * t58 + t88) * t85 + (-0.2e1 * t53 + 0.1e1) * t60 * t35;
t21 = 0.2e1 * t57 * t35 * t53 + ((-t85 * t99 + 0.2e1 * t78) * t52 + (-0.2e1 * t48 * t85 - t60 * t99) * t87 - t101 * t85 - t78) * t59 + (-t46 * t83 - t47 * t57) * t58 - t55 * (-t46 * t57 + t47 * t83);
t20 = (-0.2e1 * t103 - 0.2e1 * t38 + (-0.4e1 * t52 + 0.2e1) * t86) * t54 - t106 * t81 + t24;
t1 = [mE, 0, 0, 0, t27, t108; 0, mE, 0, -t27, 0, -t104; 0, 0, mE, -t108, t104, 0; 0, -t27, -t108, t30 * t53 + t35 * t109 + mE * (t62 + t63) + ZZ, t21, t22; t27, 0, t104, t21 (t75 + (t53 - 0.1e1) * t90) * t96 + mE * (t61 + t63) + YY + t91 * t53 + t102 - t110, t20; t108, -t104, 0, t22, t20, 0.2e1 * t76 + mE * (t61 + t62) + XX + t110;];
M = t1;

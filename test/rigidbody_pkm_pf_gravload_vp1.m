% Vektor der Gravitationskräfte der PKM-Plattform (für 6FG)
% 
% Eingabe:
% phi [3x1]
%   XYZ-Euler-Winkel des Plattform-KS
% g [3x1]
%   Gravitationsvektor im Basis-KS
% m_num [1x1], rSges_num_mdh [1x3]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Ausgabe:
% tau [6x1]
%   Vektor der Gravitationskräfte (Kraft und Moment) im Basis-KS

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

function tau = rigidbody_pkm_pf_gravload_vp1(phi, g, m_num, rSges_num_mdh)

%% Initialisierung
x_all = [zeros(3,1); phi];

mE = m_num;

r_sP = rSges_num_mdh;

g1 = g(1);
g2 = g(2);
g3 = g(3);

%% Berechnung
% Aus gravload_platform_eulxyz_matlab.m
t11 = r_sP(3);
t12 = r_sP(2);
t13 = r_sP(1);
t8 = x_all(6);
t2 = sin(t8);
t5 = cos(t8);
t14 = t12 * t2 - t13 * t5;
t9 = x_all(5);
t3 = sin(t9);
t6 = cos(t9);
t19 = -t3 * t11 + t14 * t6;
t18 = t11 * t6;
t17 = t12 * t3;
t16 = t13 * t3;
t10 = x_all(4);
t7 = cos(t10);
t4 = sin(t10);
t1 = [mE * g1; mE * g2; mE * g3; -mE * (((-g2 * t16 - g3 * t12) * t5 + g2 * t18) * t7 + ((g2 * t12 - g3 * t16) * t5 + g3 * t18) * t4 + ((g2 * t17 - g3 * t13) * t7 + (g2 * t13 + g3 * t17) * t4) * t2); mE * (t19 * g3 + ((t14 * t3 + t18) * t7 + (t12 * t5 + t2 * t13) * t4) * g1); -mE * (t19 * g2 + (-t4 * t18 + (t12 * t7 + t4 * t16) * t5 + (t13 * t7 - t4 * t17) * t2) * g1);];
tau = -t1;

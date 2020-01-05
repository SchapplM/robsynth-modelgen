% Teste die Plattform-Dynamik der PKM-Modelle gegen eine validierte Lösung
% Ziel: Prüfen, ob die Plattform-Dynamik der PKM richtig ist.
% Ergebnis: Werte stimmen überein.
% 
% Siehe auch: rigid_body_dynamics_test.m (im robotics-Repo)
% 
% Quelle:
% [Abdellatif2007] (Dissertation)
% [VorndammeSchHad2017] Vorndamme, J., Schappler, M., Haddadin, S.:
% Collision detection, isolation and identification for humanoids (ICRA17)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear
close all

% Die Dynamikfunktionen aus der Herleitung über Lagrange und Euler-Winkel
% als Minimalkoordinaten (aus Floating Base Modell ohne Gelenke) liegt im
% Robotik-Repo.
robotics_repo_path = fullfile(fileparts(which('robotics_toolbox_path_init.m')));
if isempty(robotics_repo_path)
  error('Test kann nicht ausgeführt werden: Robotik-Repo liegt nicht im Matlab-Pfad');
end

%% Einstellungen: Dynamikparameter
% Masse
m = 1.5;

% Trägheitstensor um den Schwerpunkt im Körper-KS (B)
I_B_C = inertiavector2matrix([4, 8, 15,-1,-2,-0.5]);
if any(eig(I_B_C) < 0)
  error('Gewählter Trägheitstensor ist nicht positiv definit');
end

% beliebige Schwerpunktskoordinaten (im Körper-KS B)
r_B_B_C = [.16, .23, .42]';

% Gravitation
g_W = [0;0;-9.81];

r_S = r_B_B_C';
I_S = inertiamatrix2vector(I_B_C);
I_O = inertiamatrix2vector(inertia_steiner(I_B_C, r_B_B_C, m));

% Parametervektor definieren (Inertialparameter) 
% Khalil-Notation: Reihenfolge (XX, XY, XZ, YY, YZ, ZZ, MX, MY, MZ, M)
delta = [I_O([1 4 5 2 6 3])'; m*r_S'; m];
%% Testen der Starrkörperdynamik mit verschiedenen Methoden
% Dummy-Eingaben für Dynamik-Funktionen aus Euler-XYZ-Lagrange-Herleitung
% (Hintergrund: Standard-Modell Floating-Base-Roboter ohne Gelenke)
q = zeros(0,1);
qD = zeros(0,1);
alpha_mdh = zeros(0,1);
a_mdh = zeros(0,1);
d_mdh = zeros(0,1);
q_offset_mdh = zeros(0,1);
b_mdh = zeros(0,1);
beta_mdh = zeros(0,1);

% Zufälliger Zustand der Plattform als Starrkörper
phi = rand(3,1);
xD = rand(6,1);
xDD = rand(6,1);

% Testweise berechnen der Winkel-Größen in natürlichen Einheiten (nicht
% Euler-Winkel)
omega = eulxyzD2omega(phi, xD(4:6));
omegaD = eulxyzDD2omegaD(phi, xD(4:6), xDD(4:6));
R_W_B = eulxyz2r(phi);

% Abdellatif 2007 Gl. 2.19: Matrix zur Umrechnung zwischen Euler-Winkeln
% und natürlichen Winkelgeschwindigkeiten
Tw = eulxyzjac(phi);
H = [eye(3), zeros(3,3); zeros(3,3), Tw];

% Dynamik-Berechnung aus Lagrange-Euler-XYZ-Darstellung
% Siehe z.B. [VorndammeSchHad2017], Gl. (1); Terme M_BB, C_B*qDB, g_B
c1 = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1(phi, xD, m, r_S, I_S);
M1 = rigidbody_inertiaB_floatb_eulxyz_slag_vp1(phi, m, r_S, I_S);
g1 = rigidbody_gravloadB_floatb_eulxyz_slag_vp1(phi, g_W, m, r_S);
tau1 = rigidbody_invdynB_floatb_eulxyz_slag_vp1(phi, xD, xDD, g_W, m, r_S, I_S);
tau1_sum = M1*xDD + c1 + g1;

c1t = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2(phi, xD, m, m*r_S, I_O);
M1t = rigidbody_inertiaB_floatb_eulxyz_slag_vp2(phi, m, m*r_S, I_O);
g1t = rigidbody_gravloadB_floatb_eulxyz_slag_vp2(phi, g_W, m, m*r_S);
tau1t = rigidbody_invdynB_floatb_eulxyz_slag_vp2(phi, xD, xDD, g_W, m, m*r_S, I_O);
tau1_test_sum = M1t*xDD + c1t + g1t;
if any(abs(tau1_sum-tau1_test_sum) > 1e-10) || any(abs(tau1-tau1t) > 1e-10) || any(abs(tau1-tau1_sum) > 1e-10)
  error('Herleitung Floating Base Starrkörper stimmt nicht mit Parametersatz 1 vs 2');
end

% Dynamik-Berechnung aus PKM-Plattform-Funktionen
% (aus robot_para_plattform_rotmat_dynamics.mw)
% Die Funktionen sind bezogen auf kartesische Momente und
% Winkelbeschleunigungen
M2tmp = rigidbody_pkm_pf_inertia_vp1(phi, m, r_S, I_S);
g2tmp = rigidbody_pkm_pf_gravload_vp1(phi, g_W, m, r_S);
tau2tmp = rigidbody_pkm_pf_invdyn_vp1(phi, xD, xDD, g_W, m, r_S, I_S);

% Umrechnung auf Euler-Winkel (Format der floatb_eulxyz-Funktionen)
% Die PKM-Plattform-Funktionen sind auf Momente im Basis-KS und
% Winkelbeschleunigung bezogen
g2 = H' * g2tmp; % Linkes H': g2-Momente sind in mitgedrehten Achsen definiert.
tau2 = H' * tau2tmp;
M2 = H' * M2tmp * H; % linkes H': Bezug auf Euler-Momente; rechtes H: Bezug auf Euler-Zeitableitung

% Vergleich der Berechnungen auf zwei Wege
if any(abs(g1-g2) > 1e-10)
  error('Starrkörperdynamik (Gravitation) für PKM-Plattform stimmt nicht mit eigener Berechnung überein');
end
if any(abs(M1(:)-M2(:)) > 1e-10)
  error('Starrkörperdynamik (Massenmatrix) für PKM-Plattform stimmt nicht mit eigener Berechnung überein');
end
if any(abs(tau1-tau2) > 1e-10)
  error('Starrkörperdynamik für PKM-Plattform stimmt nicht mit eigener Berechnung überein');
end
fprintf('Starrkörperdynamik der PKM-Plattform stimmt mit Euler-Winkel-Lagrange-Herleitung überein\n');

%% Teste Regressorform der Plattformdynamik (Starrkörper-Dynamik)
% Regressormatrizen aufrufen und daraus Dynamik-Funktionen berechnen
% Aus robot_para_plattform_rotmat_dynamics_regressor.mw
% Die Terme sind bezogen auf kartesische Momente.
% Die Massenmatrix ist bezogen auf Euler-Momente und Winkelbeschleunigung
taug_reg = rigidbody_pkm_pf_gravload_reg2(phi, g_W);
g3tmp = taug_reg*delta;
M_reg = rigidbody_pkm_pf_inertia_reg2(phi);
M3tmp = reshape(M_reg*delta, 6, 6);
tau_reg = rigidbody_pkm_pf_invdyn_reg2(phi, xD, xDD, g_W);
tau3tmp = tau_reg*delta;
% Umrechnung auf andere Winkeldefinitionen
g3 = H'*g3tmp; % g3tmp ist bezogen auf Moment in Welt-KS; g3 auf Moment in Euler-Winkeln
tau3 = H'*tau3tmp;
M3 = M3tmp * H; % an M3 wird von rechts die Euler-Winkel-Zeitableitung anmultipliziert; an M3tmp die Winkelgeschw.

% Vergleich der Berechnungen auf zwei Wege
if any(abs(g1-g3) > 1e-10)
  error('Starrkörperdynamik (Gravitation) für PKM-Plattform stimmt nicht mit Regressorform überein');
end
if any(abs(M1(:)-M3(:)) > 1e-10)
  error('Starrkörperdynamik (Massenmatrix) für PKM-Plattform stimmt nicht mit Regressorform überein');
end
if any(abs(tau1-tau3) > 1e-10)
  error('Starrkörperdynamik für PKM-Plattform stimmt nicht mit Regressorform überein');
end
fprintf('Starrkörperdynamik der PKM-Plattform stimmt mit Regressorform überein\n');

% Berechne Regressorform aus Lagrange
taug_reg_slag = rigidbody_gravloadB_floatb_eulxyz_reg2_slag_vp(phi, g_W);
g4tmp = taug_reg_slag*delta;
tauc_reg_slag = rigidbody_coriolisvecB_floatb_eulxyz_reg2_slag_vp(phi, xD);
c4tmp = tauc_reg_slag*delta;
tau_reg_slag = rigidbody_invdynB_floatb_eulxyz_reg2_slag_vp(phi, xD, xDD, g_W);
M_reg_slag = rigidbody_inertiaB_floatb_eulxyz_reg2_slag_vp(phi);
Mvec_slag = M_reg_slag*delta; % Massenmatrix wird als Vektor gespeichert (da symmetrisch)
M4tmp = vec2symmat(Mvec_slag);
tau4tmp = tau_reg_slag*delta;
% Teste tau gegen M+c+g
test_tau4 = tau4tmp - (M4tmp*xDD+c4tmp+g4tmp);
if any(abs(test_tau4)>1e-10)
  error('Inverse Dynamik mit Lagrange-Herleitung stimmt nicht in sich');
end

% Die Ausgaben der Floatb-Lagrange-Modellierung des Starrkörpers sind
% bezogen auf Euler-Winkel (Momente und Beschleunigungen). Keine Änderung
% notwendig bei Vergleich gegen Inertialparameterform dieser Herleitung.
g4 = g4tmp;
tau4 = tau4tmp;
M4 = M4tmp;

if any(abs(g1-g4) > 1e-10)
  error('Starrkörperdynamik (Gravitation) für PKM-Plattform stimmt nicht mit Regressorform überein');
end
if any(abs(M1(:)-M4(:)) > 1e-10)
  error('Starrkörperdynamik (Massenmatrix) für PKM-Plattform stimmt nicht mit Regressorform überein');
end
if any(abs(tau1-tau4) > 1e-10)
  error('Starrkörperdynamik für PKM-Plattform stimmt nicht mit Regressorform überein');
end
fprintf('Regressormatrix der Starrkörperdynamik mit zwei Herleitungen geprüft.\n');
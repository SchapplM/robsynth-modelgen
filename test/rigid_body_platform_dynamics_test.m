% Teste die Plattform-Dynamik der PKM-Modelle gegen eine validierte Lösung
% Ziel: Prüfen, ob die Plattform-Dynamik der PKM richtig ist.
% Ergebnis: Werte stimmen überein.
% 
% Siehe auch: rigid_body_dynamics_test.m (im robotics-Repo)

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
c1 = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp1(phi, xD, m, r_S, I_S);
M1 = rigidbody_inertiaB_floatb_eulxyz_slag_vp1(phi, m, r_S, I_S);
g1 = rigidbody_gravloadB_floatb_eulxyz_slag_vp1(phi, g_W, m, r_S);
tau1 = M1*xDD + c1 + g1;

c1t = rigidbody_coriolisvecB_floatb_eulxyz_slag_vp2(phi, xD, m, m*r_S, I_O);
M1t = rigidbody_inertiaB_floatb_eulxyz_slag_vp2(phi, m, m*r_S, I_O);
g1t = rigidbody_gravloadB_floatb_eulxyz_slag_vp2(phi, g_W, m, m*r_S);
tau1_test = M1t*xDD + c1t + g1t;
if any(abs(tau1-tau1_test) > 1e-10)
  error('Herleitung Floating Base Starrkörper stimmt nicht mit Parametersatz 1 vs 2');
end

% Dynamik-Berechnung aus PKM-Plattform-Funktionen
% (aus robot_para_plattform_rotmat_dynamics.mw)
M2tmp = rigidbody_pkm_pf_inertia_vp1(phi, m, r_S, I_S);
g2tmp = rigidbody_pkm_pf_gravload_vp1(phi, g_W, m, r_S);
tau2tmp = rigidbody_pkm_pf_invdyn_vp1(phi, xD, xDD, g_W, m, r_S, I_S);

% Umrechnung auf Euler-Winkel (Format der floatb_eulxyz-Funktionen)
% Die PKM-Plattform-Funktionen sind auf Momente im Basis-KS und
% Winkelbeschleunigung bezogen
g2 = H' * g2tmp;
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
fprintf('Starrkörperdynamik der PKM-Plattform stimmt gegen Euler-Winkel-Lagrange-Herleitung überein\n');

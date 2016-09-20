# Definitions for Robot Dynamics Code Generation
# Init
# Erstelle Definitionen für die Maple-Skripte zur Berechnung von Kinematik und Dynamik des Roboters
# 
# Quellen:
# [1] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot base inertial parameter identification: A linear matrix inequality approach (2014)
# [2] Ayusawa, K. and Venture, G. and Nakamura, Y.: Identifiability and identification of inertial parameters using the underactuated base-link dynamics for legged multibody systems (2013)
# [3] Fujimoto, Y. and Obata, S. and Kawamura, A.: Robust biped walking with active interaction control between foot and ground (1998)
# [4] Khalil, W. and Kleinfinger, J.-F.: Minimum operations and minimum parameters of the dynamic models of tree structure robots (1987)
# 
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
restart:
with(LinearAlgebra):
read "../helper/proc_MatlabExport":
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
printf("Generiere Parameter für %s\n",robot_name):
# Robotics Definitions
# Joint Angles, Velocities and Accelerations in time depending and substitution form.
# Time Depending form is for time differentiation.
# Substitution Form is for partial differentiation and code export.
qJ_t := Matrix(NQJ, 1):
qJ_s := Matrix(NQJ, 1):
qJD_s := Matrix(NQJ, 1):
qJDD_s := Matrix(NQJ, 1):
for i from 1 to NQJ do
  qJ_t(i,1):=parse(sprintf("qJ%d(t)", i)):
  qJ_s(i,1):=parse(sprintf("qJ%ds", i)):
  qJD_s(i,1):=parse(sprintf("qJD%ds", i)):
  qJDD_s(i,1):=parse(sprintf("qJDD%ds", i)):
end do:
qJD_t := diff~(qJ_t, t):
qJDD_t := diff~(qJD_t, t):
# Gravity vector in world frame
g_world := Matrix(3, 1):
g_world(1 .. 3, 1) := <g1, g2, g3>:
# Position und Orientierung der Basis. Die Orientierung ist mit XYZ-Euler-Winkeln definiert, die aber nicht im weiteren Algorithmus verwendet werden (nur Platzhalter).
# Eine Invertierung der Orientierungsdarstellung sollte nicht notwendig werden, von daher kein Problem mit Orientierungsrepräsentationssingularität.
# gem. [2], S. 4 X_base_t SE(3): Position und Orientierung.
NQB := 6:
X_base_t:=Matrix(6, 1, [rx_base(t),ry_base(t),rz_base(t),alphax_base(t), betay_base(t), gammaz_base(t)]):
X_base_s:=Matrix(6, 1, [rxs_base,rys_base,rzs_base,alphaxs_base, betays_base, gammazs_base]):
# Geschwindigkeit der Basis (pelvis)
# Twist: Verallgemeinerte Geschwindigkeit. Diese Darstellung ist nicht konsistent (Ableitung der Position stimmt nicht mit Ausdruck der Geschwindigkeit überein)
V_base_t:=Matrix(6, 1, [diff~(rx_base(t),t),diff~(ry_base(t),t),diff~(rz_base(t),t),omegax_base(t), omegay_base(t), omegaz_base(t)]):
V_base_s:=Matrix(6, 1, [vxs_base,vys_base,vzs_base,omegaxs_base, omegays_base, omegazs_base]):
# Zeitableitung davon
VD_base_t:=diff~(V_base_t,t):
VD_base_s:=Matrix(6, 1, [vDxs_base,vDys_base,vDzs_base,omegaDxs_base, omegaDys_base, omegaDzs_base]):
# Name der Methode für die Orientierungsdarstellung der Basis
base_method_name := "twist":
# Umrechnung von der Ableitung der Basis-Orientierung zu Winkelgeschwindigkeiten:
# Wird hier nicht weiter beachtet (physikalisch falsch für eine Betrachtung der Dynamik der Basis).
# Ermöglicht eine einfachere Berechnung für Mechanismen, wo die Rückwirkung auf die Basis nicht in der Vorwärtsdynamik simuliert werden muss.
T_basevel := IdentityMatrix(3, 3):
# Verallgemeinerte Koordinaten, gem [2], S. 4, [3], S.1
NQ:=NQJ+NQB:
q_t := Matrix(NQ,1, <X_base_t, qJ_t>):
q_s := Matrix(NQ,1, <X_base_s, qJ_s>):
qD_t:= Matrix(NQ,1, <V_base_t, qJD_t>):
qD_s:= Matrix(NQ,1, <V_base_s, qJD_s>):
qDD_t:= Matrix(NQ,1, <VD_base_t, qJDD_t>):
qDD_s:= Matrix(NQ,1, <VD_base_s, qJDD_s>):
# MDH-Gelenkwinkel neu speichern (Definition der verallg. Koordinaten war dort noch nicht bekannt
theta := value(theta):
# Dynamic Parameters
# Anzahl der Körper (Number of Links):
NL := NJ + 1:
# Mass of each link
M := Matrix(NL, 1):
for i from 1 to NL do
  M[i,1]:=parse(sprintf("M%d", i)):
end do:
# Center of Mass of each link (in link frame)
r_i_i_Si := Matrix(3, NL):
for i from 1 to NL do
  r_i_i_Si[1,i]:=parse(sprintf("SX%d", i)):
  r_i_i_Si[2,i]:=parse(sprintf("SY%d", i)):
  r_i_i_Si[3,i]:=parse(sprintf("SZ%d", i)):
end do:
mr_i_i_Si := Matrix(3, NL):
for i from 1 to NL do
  mr_i_i_Si[1,i]:=parse(sprintf("MX%d", i)):
  mr_i_i_Si[2,i]:=parse(sprintf("MY%d", i)):
  mr_i_i_Si[3,i]:=parse(sprintf("MZ%d", i)):
end do:
# Inertia of each link (about the center of mass, in link frame)
I_i_Si := Matrix(6, NL):
for i from 1 to NL do
  I_i_Si[1,i]:=parse(sprintf("XXC%d", i)):
  I_i_Si[2,i]:=parse(sprintf("XYC%d", i)):
  I_i_Si[3,i]:=parse(sprintf("XZC%d", i)):
  I_i_Si[4,i]:=parse(sprintf("YYC%d", i)):
  I_i_Si[5,i]:=parse(sprintf("YZC%d", i)):
  I_i_Si[6,i]:=parse(sprintf("ZZC%d", i)):
end do:
# Inertia of each link (about the origin of body frame, in link frame)
I_i_i := Matrix(6, NL):
for i from 1 to NL do
  I_i_i[1,i]:=parse(sprintf("XX%d", i)):
  I_i_i[2,i]:=parse(sprintf("XY%d", i)):
  I_i_i[3,i]:=parse(sprintf("XZ%d", i)):
  I_i_i[4,i]:=parse(sprintf("YY%d", i)):
  I_i_i[5,i]:=parse(sprintf("YZ%d", i)):
  I_i_i[6,i]:=parse(sprintf("ZZ%d", i)):
end do:
# Matrix of link inertial parameters, stacked link parameter vectors
PV2_mat := Matrix(NL, 10):
for i to NL do 
  PV2_mat[i, 1 .. 6] := I_i_i[1 .. 6, i]:
  PV2_mat[i, 7 .. 9] := mr_i_i_Si[1 .. 3, i]:
  PV2_mat[i, 10] := M[i, 1]:
end do:
# Parameter-Vektor Erstellen: vector of link inertial parameters (delta in [1]).
PV2_vec := Matrix(10*(NL), 1):
for i to NL do 
  for j to 10 do 
    PV2_vec[10*(i-1)+j] := PV2_mat[i, j]:
  end do:
end do:
# Kinematische Zwangsbedingungen
# Prüfe, ob kinematische Zwangsbedingungen in der Roboterkonfiguration genannt sind durch Prüfung der Existenz der entsprechenden Variablen.
if type( kintmp_t, 'Matrix') = false then
  kintmp_t := Matrix(1,1): # Dummy-Werte damit später alles funktioniert
  kintmp_s := Matrix(1,1):
end if:
# Export
save q_t, q_s, qD_t, qD_s, qDD_t, qDD_s, qJ_t, qJ_s, qJD_t, qJD_s, qJDD_t, qJDD_s, g_world, X_base_t, X_base_s, V_base_t, V_base_s, VD_base_t, VD_base_s, qoffset, theta, alpha, d, a,v,b,beta, M, r_i_i_Si, mr_i_i_Si, I_i_i, I_i_Si, PV2_vec, PV2_mat, robot_name, NQ,NQB,NQJ,NJ,NL, base_method_name, T_basevel, kintmp_t, kintmp_s, sprintf("../codeexport/%s_tree_floatb_twist_definitions", robot_name):
save q_t, q_s, qD_t, qD_s, qDD_t, qDD_s, qJ_t, qJ_s, qJD_t, qJD_s, qJDD_t, qJDD_s, g_world, X_base_t, X_base_s, V_base_t, V_base_s, VD_base_t, VD_base_s, qoffset, theta, alpha, d, a,v,b,beta, M, r_i_i_Si, mr_i_i_Si, I_i_i, I_i_Si, PV2_vec, PV2_mat, robot_name, NQ,NQB,NQJ,NJ,NL, base_method_name, T_basevel, kintmp_t, kintmp_s, sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Einzelne DH-Parameter als Matlab-Code exportieren. Damit lässt sich in Matlab ein passender Parametersatz generieren.
MatlabExport(v, sprintf("../codeexport/%s_parameters_mdh_v.m", robot_name), 2):
MatlabExport(a, sprintf("../codeexport/%s_parameters_mdh_a.m", robot_name), 2):
MatlabExport(d, sprintf("../codeexport/%s_parameters_mdh_d.m", robot_name), 2):
MatlabExport(b, sprintf("../codeexport/%s_parameters_mdh_b.m", robot_name), 2):
MatlabExport(alpha, sprintf("../codeexport/%s_parameters_mdh_alpha.m", robot_name), 2):
MatlabExport(beta, sprintf("../codeexport/%s_parameters_mdh_beta.m", robot_name), 2):
MatlabExport(qoffset, sprintf("../codeexport/%s_parameters_mdh_qoffset.m", robot_name), 2):


# Inverse Dynamics for Robot based on MDH frames
# Einleitung
# Berechnung der inversen Dynamik
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# worldframe -> Berechnung basierend auf Energien aus Welt-KS (KS W)
# par12 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ) oder Parametersatz 2 (1. und 2. Moment MX,MY,MZ,...)
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
restart:
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_act := true:
codegen_opt := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_Lagrange1":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen.
codegen_dynpar := 1:
# Ergebnisse der Energie laden
if codegen_dynpar = 1 then
  read sprintf("../codeexport/%s_energy_potential_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
  read sprintf("../codeexport/%s_energy_kinetic_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
elif codegen_dynpar = 2 then
  read sprintf("../codeexport/%s_energy_potential_floatb_%s_worldframe_par2_maple.m", robot_name, base_method_name):
  read sprintf("../codeexport/%s_energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
else
  printf("Dynamikfunktionen nur für Parametersatz 1 oder 2 definiert\n"):
end:
T := T:
U_grav := U_grav:
printf("Generiere Dynamik für %s mit Parametersatz %d\n", robot_name, codegen_dynpar):
# Lagrange Formalismus (mit Funktion)
OutputLagrange := Lagrange1(T, U_grav, N):
dTdqDdt_s := OutputLagrange[1]:
dTdq_s := OutputLagrange[2]:
dUdq_s := OutputLagrange[3]:
save dUdq_s, sprintf("../codeexport/%s_floatb_%s_lagrange_dUdq_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
save dTdq_s, sprintf("../codeexport/%s_floatb_%s_lagrange_dTdq_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
save dTdqDdt_s, sprintf("../codeexport/%s_floatb_%s_lagrange_dTdqDdt_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
# Extraktion einzelner Terme
# Gravitational Load
# Generate
taug_s := dUdq_s:
save taug_s, sprintf("../codeexport/%s_gravload_par%d_maple.m", robot_name, codegen_dynpar):
# Matlab Export
# Belastung der Basis (ist falsch für floatb_twist, da das Moment durch diese Wahl der verallgemeinerten Koordinaten (floatb_twist) nicht berechnet werden kann!)
# Ist korrekt für floatb_eulangrpy
if codegen_act then
  MatlabExport(taug_s(1..6), sprintf("../codeexport/%s_base_gravload_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Belastung der Gelenke
if codegen_act then
  MatlabExport(taug_s(7..N), sprintf("../codeexport/%s_joint_gravload_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Kompletter Vektor
if codegen_act then
  MatlabExport(taug_s(1..N), sprintf("../codeexport/%s_gravload_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Mass Matrix
# Generate
tauMM_s := dTdqDdt_s:
MM_s := Matrix(N, N):
for i to N do 
  for j to N do 
    MM_s[i, j] := diff(tauMM_s[i, 1], qDD_s[j, 1]):
  end do:
end do:
# Matlab Export
if codegen_act then
  MatlabExport(MM_s(1..N,1..N), sprintf("../codeexport/%s_inertia_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codegen_act then
  MatlabExport(MM_s(7..N,1..6), sprintf("../codeexport/%s_inertia_joint_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Gelenk-Massenmatrix.
# Eliminiere die Basis-Orientierung. Hat keinen Einfluss auf die Matrix, wird aber noch Maple teilweise nicht automatisch entfernt, da die Ausdrücke zu kompliziert zum Optimieren sind.
MMjj_s := MM_s(7..N,7..N):
for i from 1 to NB do
  MMjj_s := subs({X_base_s[i,1]=0},MMjj_s):
end do:
if codegen_act then
  MatlabExport(MMjj_s, sprintf("../codeexport/%s_inertia_joint_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Mass Matrix Time Derivative
# Konvertiere Massenmatrix in zeitabhängige Variablen, um Zeitableitung zu berechnen
MM_t := convert_s_t(MM_s):
MMD_t := diff~(MM_t, t):
MMD_s := convert_t_s(MMD_t):
# Matlab Export
if codegen_act then
  MatlabExport(MMD_s[1..N,1..N], sprintf("../codeexport/%s_inertia_time_derivative_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codegen_act then
  MatlabExport(MMD_s[7..N,1..6], sprintf("../codeexport/%s_inertia_joint_base_time_derivative_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
MMDjj_s := MMD_s(7..N,7..N):
for i from 1 to NB do
  MMDjj_s := subs({X_base_s[i,1]=0},MMDjj_s):
end do:
for i from 1 to 6 do
  MMDjj_s := subs({V_base_s[i,1]=0},MMDjj_s):
end do:
if codegen_act then
  MatlabExport(MMDjj_s, sprintf("../codeexport/%s_inertia_joint_joint_time_derivative_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Coriolis Vector
# Generate
tauCC_s := dTdqDdt_s-dTdq_s:
for i to N do 
  tauCC_s := subs({qDD_s(i, 1) = 0}, tauCC_s):
end do:
for i to N do 
  MatlabExport(tauCC_s(i), sprintf("../codeexport/%s_coriolisvec_floatb_%s_%d_par%d_matlab.m", robot_name, base_method_name, i, codegen_dynpar), codegen_opt):
end do:
# Matlab Export: Floating base
if codegen_act then
  MatlabExport(tauCC_s[1..6], sprintf("../codeexport/%s_coriolisvec_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codegen_act then
  MatlabExport(tauCC_s[7..N], sprintf("../codeexport/%s_coriolisvec_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codegen_act then
  MatlabExport(tauCC_s[1..N], sprintf("../codeexport/%s_coriolisvec_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
tauCC_s_fixb:=tauCC_s:
for i from 1 to NB do
  tauCC_s_fixb := subs({X_base_s[i,1]=0},tauCC_s_fixb):
end do:
for i from 1 to 6 do
  tauCC_s_fixb := subs({V_base_s[i,1]=0},tauCC_s_fixb):
end do:
if codegen_act then
  MatlabExport(tauCC_s_fixb[7..N], sprintf("../codeexport/%s_coriolisvec_joint_fixb_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:
# Coriolis Matrix
# Calculation with Christoffel Symbol approach
# [KhalilDombre2002], equ. (9.7) (p. 195)
cijk := proc (i::integer, j::integer, k::integer, A, qs)
  local c:
  c := (1/2)*(diff(A[i, j], q_s(k, 1)))+(1/2)*(diff(A[i, k], q_s(j, 1)))-(1/2)*(diff(A[j, k], q_s(i, 1))):
  return c:
end proc:
Cqs:=Matrix(N,N): 
for i  from 1 to N do
  for j from 1 to N do
    Cqs[i,j]:=0:
    for k from 1 to N do
      Cqs[i,j]:=Cqs[i,j]+cijk(i,j,k,MM_s,q_s)*qD_s[k,1]:
    end do:
  end do:
end do:
# Matlab Export: Floating base
if codegen_act then
  MatlabExport(Cqs[1..N,1..N], sprintf("../codeexport/%s_coriolismat_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codegen_act then
  MatlabExport(Cqs[7..N,1..N], sprintf("../codeexport/%s_coriolismat_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
Cqs_fixb:=Cqs:
for i from 1 to NB do
  Cqs_fixb := subs({X_base_s[i,1]=0},Cqs_fixb):
end do:
for i from 1 to 6 do
  Cqs_fixb := subs({V_base_s[i,1]=0},Cqs_fixb):
end do:
if codegen_act then
  MatlabExport(Cqs_fixb[7..N,7..N], sprintf("../codeexport/%s_coriolismat_joint_fixb_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:
# Joint Torques
tau := dTdqDdt_s-dTdq_s+dUdq_s:
# Matlab Export: Floating base
# Berechnung der Basis-Belastung ist für manche Basis-Darstellungen falsch (siehe oben unter Gravitationslast).
if codegen_act then
  MatlabExport(tau, sprintf("../codeexport/%s_invdyn_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), true):
end if:
if codegen_act then
  MatlabExport(tau[1..6], sprintf("../codeexport/%s_invdyn_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codegen_act then
  MatlabExport(tau[7..N], sprintf("../codeexport/%s_invdyn_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
taus_fixb:=tau:
for i from 1 to NB do
  taus_fixb := subs({X_base_s[i,1]=0},taus_fixb):
end do:
for i from 1 to 6 do
  taus_fixb := subs({V_base_s[i,1]=0},taus_fixb):
  taus_fixb := subs({VD_base_s[i,1]=0},taus_fixb):
end do:
if codegen_act then
  MatlabExport(taus_fixb[7..N], sprintf("../codeexport/%s_invdyn_fixb_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:


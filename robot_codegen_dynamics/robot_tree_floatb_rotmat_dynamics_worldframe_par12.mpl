
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
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_cormat := true:
codeexport_inertia := true:
codeexport_inertiaD := true:
codeexport_invdyn := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_index_symmat2vector":
read "../helper/proc_symmat2vector":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Ergebnisse des Lagrange-Formalismus laden
read sprintf("../codeexport/%s/tmp/floatb_%s_lagrange_dUdq_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
dUdq_s := dUdq_s:
read sprintf("../codeexport/%s/tmp/floatb_%s_lagrange_dTdq_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
dTdq_s := dTdq_s:
read sprintf("../codeexport/%s/tmp/floatb_%s_lagrange_dTdqDdt_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
dTdqDdt_s := dTdqDdt_s:

DynString := "Term:":
if codeexport_grav then
  DynString := sprintf("%s g",DynString):
end if:
if codeexport_corvec then
  DynString := sprintf("%s c",DynString):
end if:
if codeexport_cormat then
  DynString := sprintf("%s C",DynString):
end if:
if codeexport_inertia then
  DynString := sprintf("%s M",DynString):
end if:
if codeexport_inertiaD then
  DynString := sprintf("%s MD",DynString):
end if:
if codeexport_invdyn then
  DynString := sprintf("%s tau",DynString):
end if:
printf("Generiere Dynamik (%s) für %s mit Parametersatz %d und %s\n", DynString, robot_name, codegen_dynpar, base_method_name):
# Gravitational Load
# Generate
taug_s := dUdq_s:
save taug_s, sprintf("../codeexport/%s/tmp/gravload_par%d_maple.m", robot_name, codegen_dynpar):
# Matlab Export
# Belastung der Basis (ist falsch für floatb_twist, da das Moment durch diese Wahl der verallgemeinerten Koordinaten (floatb_twist) nicht berechnet werden kann!)
# Ist korrekt für floatb_eulxyz.
# Die Berechnungen werden deshalb nicht für floatb_twist durchgeführt.
if codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_s(1..6), sprintf("../codeexport/%s/tmp/base_gravload_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Belastung der Gelenke
if codeexport_grav then
  MatlabExport(taug_s(7..NQ), sprintf("../codeexport/%s/tmp/joint_gravload_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Kompletter Vektor
if codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_s(1..NQ), sprintf("../codeexport/%s/tmp/gravload_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Mass Matrix
# Generate
tauMM_s := dTdqDdt_s:
MM_s := Matrix(NQ, NQ):
if codeexport_inertia or codeexport_inertiaD or codeexport_cormat then
  for i to NQ do 
    for j to NQ do 
      MM_s[i, j] := diff(tauMM_s[i, 1], qDD_s[j, 1]):
    end do:
  end do:
  save MM_s, sprintf("../codeexport/%s/tmp/inertia_par%d_maple.m", robot_name, codegen_dynpar):
end if:
# Matlab Export (nur linke untere Dreiecksmatrix bei vollständigen Matrizen)
if codeexport_inertia and not(base_method_name="twist") then
  MM_s_vek := symmat2vec(MM_s):
  filename_tmp := sprintf("../codeexport/%s/tmp/inertia_floatb_%s_par%d_maple_symvec.m", robot_name, base_method_name, codegen_dynpar):
  save MM_s_vek, filename_tmp:
  read filename_tmp:
  MatlabExport(MM_s_vek, sprintf("../codeexport/%s/tmp/inertia_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MM_s(7..NQ,1..6), sprintf("../codeexport/%s/tmp/inertia_joint_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Gelenk-Massenmatrix.
# Eliminiere die Basis-Orientierung. Hat keinen Einfluss auf die Matrix, wird aber noch Maple teilweise nicht automatisch entfernt, da die Ausdrücke zu kompliziert zum Optimieren sind.
MMjj_s := MM_s(7..NQ,7..NQ):
for i from 1 to NQB do
  MMjj_s := subs({X_base_s[i,1]=0},MMjj_s):
end do:
if codeexport_inertia then
  MMjj_s_vek := symmat2vec(MMjj_s):
  filename_tmp := sprintf("../codeexport/%s/tmp/inertia_joint_joint_floatb_%s_par%d_maple_symvec.m", robot_name, base_method_name, codegen_dynpar):
  save MMjj_s_vek, filename_tmp:
  read filename_tmp:
  MatlabExport(MMjj_s_vek, sprintf("../codeexport/%s/tmp/inertia_joint_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Basis-Massenmatrix
MMbb_s := MM_s(1..6,1..6):
if codeexport_inertia and not(base_method_name="twist") then
  MMbb_s_vek := symmat2vec(MMbb_s):
  filename_tmp := sprintf("../codeexport/%s/tmp/inertia_base_base_floatb_%s_par%d_maple_symvec.m", robot_name, base_method_name, codegen_dynpar):
  save MMbb_s_vek, filename_tmp:
  read filename_tmp:
  MatlabExport(MMbb_s_vek, sprintf("../codeexport/%s/tmp/inertia_base_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Mass Matrix Time Derivative
# Konvertiere Massenmatrix in zeitabhängige Variablen, um Zeitableitung zu berechnen
MM_t := convert_s_t(MM_s):
MMD_t := diff~(MM_t, t):
MMD_s := convert_t_s(MMD_t):
# Matlab Export
if codeexport_inertiaD and not(base_method_name="twist") then
  MMD_s_vek := symmat2vec(MMD_s):
  filename_tmp := sprintf("../codeexport/%s/tmp/inertia_time_derivative_floatb_%s_par%d_maple_symvec.m", robot_name, base_method_name, codegen_dynpar):
  save MMD_s_vek, filename_tmp:
  read filename_tmp:
  MatlabExport(MMD_s_vek, sprintf("../codeexport/%s/tmp/inertia_time_derivative_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_inertiaD and not(base_method_name="twist") then
  MatlabExport(MMD_s[7..NQ,1..6], sprintf("../codeexport/%s/tmp/inertia_joint_base_time_derivative_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
MMDjj_s := MMD_s(7..NQ,7..NQ):
for i from 1 to NQB do
  MMDjj_s := subs({X_base_s[i,1]=0},MMDjj_s):
end do:
for i from 1 to 6 do
  MMDjj_s := subs({V_base_s[i,1]=0},MMDjj_s):
end do:
if codeexport_inertiaD then
  # Vektor der unteren linken Dreiecksmatrix generieren, speichern und wieder laden. Ohne Neuladen hängt sich maple durch einen Bug auf. TODO: Klären.
  MMDjj_s_vek := symmat2vec(MMDjj_s):
  filename_tmp := sprintf("../codeexport/%s/tmp/inertia_joint_joint_time_derivative_floatb_%s_par%d_maple_symvec.m", robot_name, base_method_name, codegen_dynpar):
  save MMDjj_s_vek, filename_tmp:
  read filename_tmp:
  MatlabExport(MMDjj_s_vek, sprintf("../codeexport/%s/tmp/inertia_joint_joint_time_derivative_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Coriolis Vector
# Generiere Coriolis-Vektor aus dem Lagrange-Momenten
tauCC_s := Matrix(NQ,1): # Platzhalter
;
if codeexport_corvec then
  tauCC_s := dTdqDdt_s-dTdq_s:
  for i to NQ do 
    tauCC_s := subs({qDD_s(i, 1) = 0}, tauCC_s):
  end do:
  save tauCC_s, sprintf("../codeexport/%s/tmp/coriolisvec_par%d_maple.m", robot_name, codegen_dynpar):
end if:
# Einzelne Komponenten exportieren (falls der ganze Vektor zu lange dauert).
if codeexport_corvec and not(base_method_name="twist") and codegen_debug then
  for i to NQ do 
    MatlabExport(tauCC_s(i), sprintf("../codeexport/%s/tmp/coriolisvec_floatb_%s_%d_par%d_matlab.m", robot_name, base_method_name, i, codegen_dynpar), codegen_opt):
  end do:
end if:
# Matlab Export: Floating base
if codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauCC_s[1..6], sprintf("../codeexport/%s/tmp/coriolisvec_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauCC_s[7..NQ], sprintf("../codeexport/%s/tmp/coriolisvec_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauCC_s[1..NQ], sprintf("../codeexport/%s/tmp/coriolisvec_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
tauCC_s_fixb:=tauCC_s:
for i from 1 to NQB do
  tauCC_s_fixb := subs({X_base_s[i,1]=0},tauCC_s_fixb):
end do:
for i from 1 to 6 do
  tauCC_s_fixb := subs({V_base_s[i,1]=0},tauCC_s_fixb):
end do:
if codeexport_corvec then
  MatlabExport(tauCC_s_fixb[7..NQ], sprintf("../codeexport/%s/tmp/coriolisvec_joint_fixb_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:
# Coriolis Matrix
# Calculation with Christoffel Symbol approach
# [KhalilDombre2002], equ. (9.7) (p. 195)
cijk := proc (i::integer, j::integer, k::integer, A, qs)
  local c:
  c := (1/2)*(diff(A[i, j], qs(k, 1)))+(1/2)*(diff(A[i, k], qs(j, 1)))-(1/2)*(diff(A[j, k], qs(i, 1))):
  return c:
end proc:
if codeexport_cormat then
  Cqs:=Matrix(NQ,NQ): 
  for i  from 1 to NQ do
    for j from 1 to NQ do
      Cqs[i,j]:=0:
      for k from 1 to NQ do
        Cqs[i,j]:=Cqs[i,j]+cijk(i,j,k,MM_s,q_s)*qD_s[k,1]:
      end do:
    end do:
  end do:
end if:
# Matlab Export: Floating base
if codeexport_cormat and not(base_method_name="twist") then
  MatlabExport(Cqs[1..NQ,1..NQ], sprintf("../codeexport/%s/tmp/coriolismat_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_cormat and not(base_method_name="twist") then
  MatlabExport(Cqs[7..NQ,1..NQ], sprintf("../codeexport/%s/tmp/coriolismat_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
Cqs_fixb:=Cqs:
if codeexport_cormat then
  for i from 1 to NQB do
    Cqs_fixb := subs({X_base_s[i,1]=0},Cqs_fixb):
  end do:
  for i from 1 to 6 do
    Cqs_fixb := subs({V_base_s[i,1]=0},Cqs_fixb):
  end do:
end if:
if codeexport_cormat then
  MatlabExport(Cqs_fixb[7..NQ,7..NQ], sprintf("../codeexport/%s/tmp/coriolismat_joint_fixb_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:
# Joint Torques
if codeexport_invdyn then
  tau := dTdqDdt_s-dTdq_s+dUdq_s:
end if:
# Matlab Export: Floating base
# Berechnung der Basis-Belastung ist für manche Basis-Darstellungen falsch (siehe oben unter Gravitationslast).
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau[1..6], sprintf("../codeexport/%s/tmp/invdyn_base_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau[7..NQ], sprintf("../codeexport/%s/tmp/invdyn_joint_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
if codeexport_invdyn then
  taus_fixb:=tau:
  for i from 1 to NQB do
    taus_fixb := subs({X_base_s[i,1]=0},taus_fixb):
  end do:
  for i from 1 to 6 do
    taus_fixb := subs({V_base_s[i,1]=0},taus_fixb):
    taus_fixb := subs({VD_base_s[i,1]=0},taus_fixb):
  end do:
  save taus_fixb, sprintf("../codeexport/%s/tmp/invdyn_fixb_par%d_maple.m", robot_name, codegen_dynpar):
end if:
if codeexport_invdyn then
  MatlabExport(taus_fixb[7..NQ], sprintf("../codeexport/%s/tmp/invdyn_fixb_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:


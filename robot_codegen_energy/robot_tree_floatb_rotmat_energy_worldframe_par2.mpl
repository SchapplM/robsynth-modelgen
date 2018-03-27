
# Energy Calculation for Atlas Robot based on MDH frames
# Introduction
# Berechnung von potentieller Energie für den Roboter.
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# energy -> Berechnung der Energie
# worldframe -> Berechnung der Positionen und Geschwindigkeiten im Welt-KS (KS W) anstatt im Basis-KS (KS 0)
# par2 -> Parametersatz 2 (erstes Moment anstelle von Schwerpunkt als Parameter)

# Anmerkung
# Nur potentielle Energie möglich
# Kinetische Energie stimmt nicht überein. Funktioniert nicht mit diesem Parametervektor und Geschwindigkeiten im Basis-KS
# Für die kinetische Energie sollte robot_tree_floatb_twist_rotmat_energy_linkframe_par2.mw genommen werden!
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
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
codegen_act := true:
codegen_opt := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
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
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf_c := Trf_c:
read sprintf("../codeexport/%s/tmp/kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
r_W_i_Si := r_W_i_Si:
mr_W_i_Si := mr_W_i_Si:
# Potential Energy
U_b := Matrix(NL, 1):
U_grav := 0:
for i to NL do 
  r_W_W_i := Matrix(3,1,Trf_c(1 .. 3, 4, i)):
  U_b[i] := -Multiply(Transpose(g_world), M[i, 1]*r_W_W_i+Matrix(3,1,mr_W_i_Si(1 .. 3, i))):
  U_grav := U_grav+U_b[i, 1]:
  printf("Potentielle Energie aus Gravitation für Körper %d berechnet (im Welt-KS, mit Parametersatz 2).\n", i-1):#0=Basis
end do:
# Maple Export
save U_grav, sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par2_maple.m", robot_name, base_method_name):
# Kinetic Energy
# Using velocities in world frame and MX,MY,MZ as a parameter does not work.
# Only calculation of potential energy in this worksheet
# Matlab Export
# Potential Energy
# Floating Base
U_s := convert_t_s(U_grav):
if codegen_act then
  MatlabExport(U_s, sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par2_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Fixed Base
U_s_fixb:=U_s:
for i from 1 to NQB do
  U_s_fixb := subs({X_base_s[i,1]=0},U_s_fixb):
end do:
for i from 1 to 6 do
  U_s_fixb := subs({V_base_s[i,1]=0},U_s_fixb):
end do:
if codegen_act then
  MatlabExport(U_s_fixb, sprintf("../codeexport/%s/tmp/energy_potential_fixb_worldframe_par2_matlab.m", robot_name), codegen_opt):
end if:


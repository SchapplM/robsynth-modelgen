# Energy Calculation for the Robot links based on MDH frames
# Introduction
# Berechnung von kinetischer Energie für den Roboter.
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# energy -> Berechnung der Energie
# linkframe -> Berechnung der Geschwindigkeit im Körper-KS (KSi)
# par2 -> Parametersatz 2 (erstes Moment anstelle von Schwerpunkt als Parameter)
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1988] A direct determination of minimum inertial parameters of robots
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Craig] Introduction to Robotics, Modeling and Control (3rd Edition)
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
printf("Generiere Geschwindigkeit für %s\n", robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name, base_method_name):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf_c := Trf_c:
read sprintf("../codeexport/%s/tmp/velocity_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
omega_i_i := omega_i_i: 
rD_i_i := rD_i_i:
# Calculate Energy
# Use velocities in link frame and MX,MY,MZ as a parameter
# Kinetic Energy
# [GautierKhalil1988], equ.6
# Hier wird der Trägheitstensor um den Koordinatenursprung gewählt, und nicht um den Schwerpunkt!
T_b := Matrix(NL, 1):
T := 0:
for i to NL do 
  I_i_i_tensor := Matrix([[I_i_i[1, i], I_i_i[2, i], I_i_i[3, i]], [I_i_i[2, i], I_i_i[4, i], I_i_i[5, i]], [I_i_i[3, i], I_i_i[5, i], I_i_i[6, i]]]):
  T_b_rot := (1/2)*Multiply(Transpose(omega_i_i(1 .. 3, i)), Multiply(I_i_i_tensor, omega_i_i(1 .. 3, i))):
  T_b_trans := (1/2)*M[i, 1]*Multiply(Transpose(rD_i_i(1 .. 3, i)), rD_i_i(1 .. 3, i))+Multiply(Transpose(rD_i_i(1 .. 3, i)), CrossProduct(omega_i_i(1 .. 3, i), mr_i_i_Si(1 .. 3, i))):
  T_b[i, 1] := T_b_rot+T_b_trans:
  T := T+T_b[i, 1]:
end do:
# Maple Export
save T, sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
# Potential Energy
# Berechnung in Welt-KS ist bereits erfolgreich. Wird hier nicht nochmal durchgeführt.
# siehe robot_tree_floatb_twist_rotmat_energy_baseframe_par2.mw
# Export
# Matlab Export
# Floating Base
# Eliminiere die Basis-Position (nicht: Orientierung). Die kinetische Energie kann nicht davon abhängen.
# Falls die Ausdrücke doch vorkommen, liegt es daran, dass Maple diese nicht wegoptimieren konnte.
T_s:=convert_t_s(T):
for i from 1 to 3 do
  T_s := subs({X_base_s[i,1]=0},T_s):
end do:
if codegen_act then
  MatlabExport(T_s, sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_linkframe_par2_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Fixed Base
T_s_fixb:=T_s:
for i from 1 to NQB do
  T_s_fixb := subs({X_base_s[i,1]=0},T_s_fixb):
end do:
for i from 1 to 6 do
  T_s_fixb := subs({V_base_s[i,1]=0},T_s_fixb):
end do:
if codegen_act then
  MatlabExport(T_s_fixb, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_linkframe_par2_matlab.m", robot_name), codegen_opt):
end if:


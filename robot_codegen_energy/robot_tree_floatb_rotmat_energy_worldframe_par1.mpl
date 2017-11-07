# Energy Calculation for the Robot based on MDH frames
# Introduction
# Berechnung von potentieller und kinetischer Energie für den Roboter.
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# energy -> Berechnung der Energie
# worldframe -> Berechnung der Positionen und Geschwindigkeiten im Welt-KS (KS W) anstatt im Basis-KS (KS 0)
# par1 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ)
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I (WS 2014/15)
# [Ott2008] Cartesian Impedance Control of Redundant and Flexible-Joint Robots
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
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Ergebnisse der Kinematik laden (Rotationsmatrizen, Schwerpunktskoordinaten, Geschwindigkeiten)
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
read sprintf("../codeexport/%s/tmp/kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
r_W_W_Si := r_W_W_Si:
r_W_i_Si := r_W_i_Si:
read sprintf("../codeexport/%s/tmp/velocity_worldframe_floatbase_%s_par1_maple.m", robot_name, base_method_name):
omega_W_i := omega_W_i: 
rD_W_i := rD_W_i:
rD_W_Si := rD_W_Si:
# Potential Energy
U_b := Matrix(NL, 1):
U_grav := 0:
for i to NL do 
  U_b[i] := -M[i, 1]*Multiply(Transpose(g_world), r_W_W_Si(1 .. 3, i)):
  U_grav := U_grav+U_b[i, 1]:
  printf("Potentielle Energie aus Gravitation für Körper %d berechnet\n", i):
end do:
# Maple Export
save U_grav, sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
# Kinetic Energy
# Use velocities in base frame and SX,SY,SZ as a parameter
# Berechne kinetische Energie aller durch Gelenkwinkel bewegter Körper.
T_b := Matrix(NL, 1):
T := 0:
for i to NL do 
  # Trägheitstensor (3x3) um den Körperschwerpunkt in Körper-KS
  I_i_Si_Tensor := Matrix([[I_i_Si[1, i], I_i_Si[2, i], I_i_Si[3, i]], [I_i_Si[2, i], I_i_Si[4, i], I_i_Si[5, i]], [I_i_Si[3, i], I_i_Si[5, i], I_i_Si[6, i]]]):
  # Rotation vom Welt-KS zum jeweiligen Körper-KS (Die Basis ist dabei laufende Nummer 1)
  R_W_i := Matrix(Trf_c(1 .. 3, 1 .. 3, i)):
  # Trägheitstensor (3x3) um den Körperschwerpunkt in Basis-KS
  I_i_W := Multiply(R_W_i, Multiply(I_i_Si_Tensor, Transpose(R_W_i))):
  T_b[i] := (1/2)*M[i, 1]*Multiply(Transpose(rD_W_Si(1 .. 3, i)), rD_W_Si(1 .. 3, i))+(1/2)*Multiply(Transpose(omega_W_i(1 .. 3, i)), Multiply(I_i_W, omega_W_i(1 .. 3, i))):
  T := T+T_b[i, 1]:
  printf("Kinetische Energie für Körper %d berechnet\n", i):
end do:
# Maple Export
save T, sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
# Matlab Export
# Potential Energy
# Floating Base
U_s:=convert_t_s(U_grav):
if codegen_act then
  MatlabExport(U_s, sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par1_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Fixed Base
U_s_fixb:=U_s:
for i from 1 to NQB do
  U_s_fixb := subs({X_base_s[i,1]=0},U_s_fixb):
end do:
if codegen_act then
  MatlabExport(U_s_fixb, sprintf("../codeexport/%s/tmp/energy_potential_fixb_worldframe_par1_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Kinetic Energy
# Floating Base
# Eliminiere die Basis-Position (nicht: Orientierung). Die kinetische Energie kann nicht davon abhängen.
# Falls die Ausdrücke doch vorkommen, liegt es daran, dass Maple diese nicht wegoptimieren konnte.
T_s:=convert_t_s(T):
for i from 1 to 3 do
  T_s := subs({X_base_s[i,1]=0},T_s):
end do:
if codegen_act then
  MatlabExport(T_s, sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_worldframe_par1_matlab.m", robot_name, base_method_name), codegen_opt):
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
  MatlabExport(T_s_fixb, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_worldframe_par1_matlab.m", robot_name), codegen_opt):
end if:


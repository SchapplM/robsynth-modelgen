# Center of Mass Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Schwerpunkts-Kinematik (Positionen, Jacobi)
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# kinematics_com -> Kinematik der Körperschwerpunkte
# worldframe -> Berechnung der Geschwindigkeit im Welt-KS (KSW)
# par1 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ)
# Testergebnisse
# Test des exportierten Matlab-Codes erfolgreich (kinetische Energie)

# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I (WS 2014/15)
# [Ott2008] Cartesian Impedance Control of Redundant and Flexible-Joint Robots
# [SugiharaNakIno2002] Sugihara, T. and Nakamura, Y. and Inoue, H.: Real-time humanoid motion generation through ZMP manipulation based on inverted pendulum control (2002)
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
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
printf("Generiere Schwerpunktskinematik für %s\n", robot_name):
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s_kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Positions of Center of Mass
r_W_i_Si := Matrix(3, NL):
r_W_W_Si := Matrix(3, NL):
mr_W_i_Si := Matrix(3, NL):
# Es gibt NL Körper (Basis und NJ bewegliche Körper).
# r_i_i_Si(1 .. 3, i) ist der Vektor vom Ursprung des Körper-KS i zum Schwerpunkt des Körpers i ausgedrückt im Körper-KS i
# r_W_i_Si(1 .. 3, i) ist der Vektor vom Ursprung des Körper-KS i zum Schwerpunkt des Körpers i ausgedrückt im Basis-KS
# mr_W_i_Si(1 .. 3, i) ist der Vektor vom Ursprung des Körper-KS i zum Schwerpunkt des Körpers i ausgedrückt im Basis-KS multipliziert mit der Masse des Segments i 
# r_W_W_Si(1 .. 3, i) ist der Vektor vom Ursprung des Basis-KS zum Schwerpunkt des Körpers i ausgedrückt im Basis-KS
# Trf_c(1 .. 3, 1 .. 3, i) ist das Körperkoordinatensystem zu Körper i im Basis KS.
for i to NL do
  r_W_i_Si(1 .. 3, i) :=  Multiply(Matrix(Trf_c(1 .. 3, 1 .. 3, i)), r_i_i_Si(1 .. 3, i)):
  mr_W_i_Si(1 .. 3, i) := Multiply(Matrix(Trf_c(1 .. 3, 1 .. 3, i)), mr_i_i_Si(1 .. 3, i)):
  r_W_W_Si(1 .. 3, i) :=  Matrix(Trf_c(1 .. 3, 4, i)) + Matrix(r_W_i_Si(1 .. 3, i)):
  printf("Schwerpunktsposition in Weltkoordinaten für Körper %d aufgestellt.\n", i):
end do:
# Maple Export
save mr_W_i_Si, r_W_W_Si, r_W_i_Si, sprintf("../codeexport/%s_kinematics_com_worldframe_floatb_%s_par1_maple", robot_name, base_method_name):
save mr_W_i_Si, r_W_W_Si, r_W_i_Si, sprintf("../codeexport/%s_kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
printf("Maple-Ausdrücke exportiert.\n"):
# Matlab-Export
if codegen_act then
  MatlabExport(convert_t_s(r_W_W_Si), sprintf("../codeexport/%s_fkine_com_floatb_%s_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Calculate CoM of the whole robot
c:=Vector(3):
M_ges:=0:
for i from 1 to NL do
  c:=c + r_W_W_Si(1 .. 3,i)*M[i,1]:
  M_ges:=M_ges + M[i,1]:
end do:
c:=c/M_ges:
# Maple-Export
save c, sprintf("../codeexport/%s_kinematics_com_total_worldframe_floatb_%s_par1_maple", robot_name, base_method_name):
# Matlab-Export
if codegen_act then
  MatlabExport(convert_t_s(c), sprintf("../codeexport/%s_com_total_worldframe_floatb_%s_par1_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Calculate CoG Jacobian
# CoG Jacobian nach [SugiharaNakIno2002].
# TODO: baseframe in worldframe ändern
J_COG_s := Matrix(3, NJ):
c_s := convert_t_s(c):
for i from 1 to 3 do
  for j from 1 to NJ do
    J_COG_s(i,j) := diff(c_s(i,1), qJ_s(j,1)):
  end:
end:
# TODO: Wenn der Ausdruck J_COG_s nicht gespeichert und wieder geladen wird, hängt die Berechnung bei der Optimierung des Ausdrucks mit "tryhard"
save J_COG_s, sprintf("../codeexport/%s_kinematics_Jcom_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
read sprintf("../codeexport/%s_kinematics_Jcom_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
if codegen_act then
  MatlabExport(J_COG_s, sprintf("../codeexport/%s_com_jacobi_baseframe_par1_matlab.m", robot_name), 2):
end if:
# CoG-Jacobian Time derivative
J_COG := convert_s_t(J_COG_s):
JD_COG := diff~(J_COG, t):
JD_COG_s := convert_t_s(JD_COG):
if codegen_act then
  MatlabExport(convert_t_s(JD_COG_s), sprintf("../codeexport/%s_com_jacobiD_baseframe_par1_matlab.m", robot_name), codegen_opt):
end if:


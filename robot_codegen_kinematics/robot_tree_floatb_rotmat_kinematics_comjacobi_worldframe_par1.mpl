# Center of Mass Jacobian for the Robot based on MDH frames
# Introduction
# Berechnung der Schwerpunkts-Jacobi-Matrix (Veränderung des Schwerpunktes in Abhängigkeit der Gelenkwinkel).
# Diese Matrix wird bei Gleichgewichtsregelungen (z.B. zweibeiniges Laufen) gebraucht.
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# kinematics_comjacobi -> Jacobi-Matrix der Körperschwerpunkte
# worldframe -> Berechnung der Geschwindigkeit im Welt-KS (KSW)
# par1 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ)
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2015-12
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
# Sources
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
# Schwerpunkt des Roboters laden
read sprintf("../codeexport/%s_kinematics_com_total_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
# Calculate CoG Jacobian
# CoG Jacobian nach [SugiharaNakIno2002].
# TODO: baseframe in worldframe ändern
J_COG_s := Matrix(3, NQJ):
c_s := convert_t_s(c):
for i from 1 to 3 do
  for j from 1 to NQJ do
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


# Calculate Jacobi-Matrix based on MDH-Frames
# Introduction
# Jacobi-Matrix für Roboterkinematik basierend auf MDH-Parametern berechnen.
# Die Berechnung erfolgt durch symbolische Ableitung der Transformationsmatrizen für jeden Körper
# 
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# jacobian -> Berechnung der Jacobi-Matrix
# baseframe -> Berechnung der Jacobi-Matrix bezogen auf das Basis-Koordinatensystem des Roboters (nicht: Welt-KS)

# Tests
# 
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
# Sources
# [Ortmaier2014] Vorlesungsskript Robotik I
# [Ortmaier2014a] Vorlesungsskript Robotik II
# Initialization
restart:
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
printlevel := 2:
codegen_act := true:
codegen_opt := 2:
# Funktionen aus IRT-Maple-Repo
read sprintf("../helper/proc_MatlabExport", maple_repo_path):
read sprintf("../transformation/proc_r2rpy", maple_repo_path):
read sprintf("../transformation/proc_transl", maple_repo_path):
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tree_floatb_definitions", robot_name):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden.
LIJAC:=NL:
printf("Generiere Jacobi-Matrix für %s (Körper %d)\n", robot_name, LIJAC):
# Jacobi-Matrix analytisch (Translatorisch)
# Ortmaier2014a Gl. (1.15), S.14: Geometrische Zwangsbedingungen in impliziter Form
# Gleichung enthält verallgemeinerte Koordinaten und Endeffektorposition und ergibt Null.
x_EE := Matrix(3,1,<r_xEE; r_yEE; r_zEE>):
# Transformationsmatrix von der Basis zum gegebenen Punkt auf dem Körper. Die Koordinaten px,py,pz für den Punkt werden später eingesetzt.
T_p := Trf_c(1 .. 4, 1..4, LIJAC) . transl(<px;py;pz>):
h_transl := Matrix(T_p(1..3,4)) - x_EE:
h_transl_s :=convert_t_s(h_transl):
# Jacobi-Matrix der inversen Kinematik
b_transl := Matrix(3, NQJ):
for i from 1 to 3 do
  for j from 1 to NQJ do
    b_transl(i,j) := diff(h_transl_s(i,1), qJ_s(j,1)):
  end do:
end do:
# Export
save b_transl, sprintf("../codeexport/%s/jacobia_transl_maple.m", robot_name):
# Ausdruck muss nochmal geladen werden, ansonsten hängt sich die Code-Optimerung mit "tryhard" auf.
read sprintf("../codeexport/%s/jacobia_transl_maple.m", robot_name):
b_transl := b_transl:
for i from 1 to 3 do
  for j from 1 to NQJ do
    MatlabExport(b_transl(i,j), sprintf("../codeexport/%s/jacobia_transl_%d_floatb_%s_%d_%d_matlab.m", robot_name, LIJAC, base_method_name, i, j), codegen_opt):
  end do:
end do:
MatlabExport(b_transl, sprintf("../codeexport/%s/jacobia_transl_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
# Jacobi-Matrix analytisch (Rotatorisch)
# Ortmaier2014a Gl. (1.15), S.14: Geometrische Zwangsbedingungen in impliziter Form
# Rotationsdarstellung des Endeffektors in RPY-Winkeln
xo_EE := Matrix(3,1,<phi_xEE; phi_yEE; phi_zEE>):
# r2rpy liefert inert-arctan (%). Auswertung vor Export dauert zu lange. In exportiertem Matlab-Code muss "%arctan" händisch gegen "atan2" getauscht werden.
h_rota_rpy := r2rpy(T_p) - xo_EE: 
h_rota_s :=convert_t_s(h_rota_rpy):
# Jacobi-Matrix der inversen Kinematik
# Falls die RPY-Darstellung für dieses Körper-KS singulär ist, existiert die analytische Jacobi-Matrix nicht. Maple gibt dann einen Fehler aus. Die Kennzeichnung mit NaN sorgt dafür, dass dies in Matlab erkennbar ist.
# Die Singularität kann bei kinematischen Strukturen am Anfang auftreten.
b_rota := Matrix(3, NQJ):
for i from 1 to 3 do
  for j from 1 to NQJ do
    if T_p(3,3) = 0 then
      b_rota(i,j) := NaN: # Singulär
    else
      b_rota(i,j) := diff(h_rota_s(i,1), qJ_s(j,1)):
    end if:
  end do:
end do:
# Export
save b_rota, sprintf("../codeexport/%s/jacobia_rot_maple.m", robot_name):
# Ausdruck nochmal laden.
read sprintf("../codeexport/%s/jacobia_rot_maple.m", robot_name):
b_rota := b_rota:
for i from 1 to 3 do
  for j from 1 to NQJ do
    MatlabExport(b_rota(i,j), sprintf("../codeexport/%s/jacobia_rot_%d_floatb_%s_%d_%d_matlab.m", robot_name, LIJAC, base_method_name, i, j), codegen_opt):
  end do:
end do:
MatlabExport(b_rota, sprintf("../codeexport/%s/jacobia_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
# Jacobi-Matrix geometrisch (Rotatorisch)
# Zusammenhang zwischen Geschwindigkeit der verallgemeinerten Koordinaten und Winkelgeschwindigkeit des Endeffektors ausgedrückt im Basis-Koordinatensystems
read sprintf("../codeexport/%s/velocity_worldframe_floatbase_%s_par1_maple.m", robot_name, base_method_name):
omega_W_i := omega_W_i:
omega_EE := Matrix(3,1,<omega_xEE; omega_yEE; omega_zEE>):
h_rotg := Matrix(omega_W_i(1..3,LIJAC)) - omega_EE:
h_rotg_s :=convert_t_s(h_rotg):
# Jacobi-Matrix der inversen Kinematik
# Leite die Winkelgeschwindigkeiten des Endeffektors nach den Gelenkwinkelgeschwindigkeiten ab.
# Damit wird der Zusammenhang nach Ortmaier2014 Gl. (4.17), S.51 umgeformt.
b_rotg := Matrix(3, NQJ):
for i from 1 to 3 do
  for j from 1 to NQJ do
    b_rotg(i,j) := diff(h_rotg_s(i,1), qJD_s(j,1)):
  end do:
end do:
B_s := b_rotg:
# Export
save b_rotg, sprintf("../codeexport/%s/jacobig_rot_maple.m", robot_name):
# Ausdruck nochmal laden.
read sprintf("../codeexport/%s/jacobig_rot_maple.m", robot_name):
b_rotg := b_rotg:
for i from 1 to 3 do
  for j from 1 to NQJ do
    MatlabExport(b_rotg(i,j), sprintf("../codeexport/%s/jacobig_rot_%d_floatb_%s_%d_%d_matlab.m", robot_name, LIJAC, base_method_name, i, j), codegen_opt):
  end do:
end do:
MatlabExport(b_rotg, sprintf("../codeexport/%s/jacobig_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
# Jacobi-Zeitableitung
JD_rotg_t := diff~(convert_s_t(b_rotg), t):
JD_rotg_s := convert_t_s(JD_rotg_t):
JD_rota_t := diff~(convert_s_t(b_rota), t):
JD_rota_s := convert_t_s(JD_rota_t):
JD_transla_t := diff~(convert_s_t(b_transl), t):
JD_transla_s := convert_t_s(JD_transla_t):
MatlabExport(JD_rotg_s, sprintf("../codeexport/%s/jacobigD_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
MatlabExport(JD_rota_s, sprintf("../codeexport/%s/jacobiaD_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
MatlabExport(JD_transla_s, sprintf("../codeexport/%s/jacobiaD_transl_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):


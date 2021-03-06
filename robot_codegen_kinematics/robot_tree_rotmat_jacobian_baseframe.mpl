
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
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
# Sources
# [SchapplerTapOrt2019] Schappler, M. et al.: Resolution of Functional Redundancy for 3T2R Robot Tasks using Two Sets of Reciprocal Euler Angles, Proc. of the 15th IFToMM World Congress, 2019
# [Ortmaier2014] Vorlesungsskript Robotik I
# [Ortmaier2014a] Vorlesungsskript Robotik II
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
codegen_debug := false:
codegen_opt := 2:
codegen_jacobi := 3: # 1=nur Position, 2=Rotationsmatrix-Jacobi, 3=mit Zeitableitung
;
# Funktionen aus Robotik-Repo
read("../robotics_repo_path"): # Lädt Variable robotics_repo_path
;
read(sprintf("%s/transformation/maple/proc_r2eulxyz_inert", robotics_repo_path)):
# Funktionen aus diesem Repo
read sprintf("../helper/proc_MatlabExport"):
read sprintf("../transformation/proc_transl"):
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s":
# Roboter-Definitionen
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Eliminiere die Basis-Position und Orientierung. Darf keinen Einfluss auf die Jacobi-Matrix haben.
# Bei komplizierten Ausdrücken (insbesondere bei kinematischen Zwangsbedingungen) können die Symbole manchmal nicht eliminiert werden
for i from 1 to NQB do
  Trf_c := subs({X_base_t[i,1]=0},Trf_c):
end do:
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
printf("Generiere Jacobi-Matrix für %s (Körper %d)\n", robot_name, LIJAC):
# Jacobi-Matrix analytisch (Translatorisch)
# Ortmaier2014a Gl. (1.15), S.14: Geometrische Zwangsbedingungen in impliziter Form
# Gleichung enthält verallgemeinerte Koordinaten und Endeffektorposition und ergibt Null.
x_EE := Matrix(3,1,<r_xEE; r_yEE; r_zEE>):
# Transformationsmatrix von der Basis zum gegebenen Punkt auf dem Körper. Die Koordinaten px,py,pz für den Punkt werden später eingesetzt.
T_p := Trf_c(1 .. 4, 1..4, LIJAC+1) . transl(<px;py;pz>):
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
save b_transl, sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", robot_name, LIJAC):
# Ausdruck muss nochmal geladen werden, ansonsten hängt sich die Code-Optimerung mit "tryhard" auf.
# TODO: Klären warum das so ist und Problem beheben.
read sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", robot_name, LIJAC):
b_transl := b_transl:
if codegen_act and codegen_debug then
  for i from 1 to 3 do
    for j from 1 to NQJ do
      MatlabExport(b_transl(i,j), sprintf("../codeexport/%s/tmp/jacobia_transl_%d_floatb_%s_%d_%d_matlab.m", robot_name, LIJAC, base_method_name, i, j), codegen_opt):
    end do:
  end do:
end if:
if codegen_act then
  MatlabExport(b_transl, sprintf("../codeexport/%s/tmp/jacobia_transl_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
end if:
# Jacobi-Matrix analytisch (Rotatorisch)
# Ortmaier2014a Gl. (1.15), S.14: Geometrische Zwangsbedingungen in impliziter Form
# Rotationsdarstellung des Endeffektors in RPY-Winkeln
xo_EE := Matrix(3,1,<phi_xEE; phi_yEE; phi_zEE>):
# r2eulxyz_inert liefert inert-arctan (%). Auswertung vor Export dauert zu lange. In exportiertem Matlab-Code muss "%arctan" händisch gegen "atan2" getauscht werden.
h_rota_rpy := r2eulxyz_inert(T_p) - xo_EE: 
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
save b_rota, sprintf("../codeexport/%s/tmp/jacobia_rot_%d_maple.m", robot_name, LIJAC):
# Ausdruck nochmal laden.
read sprintf("../codeexport/%s/tmp/jacobia_rot_%d_maple.m", robot_name, LIJAC):
b_rota := b_rota:
# Einzelne Ausdrücke als Code exportieren (falls Gesamtausdruck zu komplex ist)
if codegen_act and codegen_debug then
  for i from 1 to 3 do
    for j from 1 to NQJ do
      MatlabExport(b_rota(i,j), sprintf("../codeexport/%s/tmp/jacobia_rot_%d_floatb_%s_%d_%d_matlab.m", robot_name, LIJAC, base_method_name, i, j), codegen_opt):
    end do:
  end do:
end if:
if codegen_act then
  interface(warnlevel=0): # Unterdrücke die Warnung für arctan
  MatlabExport(b_rota, sprintf("../codeexport/%s/tmp/jacobia_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
  interface(warnlevel=3):
end if:
# Jacobi-Matrix geometrisch (Rotatorisch)
# Zusammenhang zwischen Geschwindigkeit der verallgemeinerten Koordinaten und Winkelgeschwindigkeit des Endeffektors ausgedrückt im Basis-Koordinatensystems
read sprintf("../codeexport/%s/tmp/velocity_worldframe_floatbase_%s_par1_maple.m", robot_name, base_method_name):
omega_W_i := omega_W_i:
# Basis-Terme entfernen (falls durch Maple später nicht möglich, s.o.)
for i from 1 to NQB do
  omega_W_i := subs({X_base_t[i,1]=0},omega_W_i):
  omega_W_i := subs({V_base_t[i,1]=0},omega_W_i):
end do:
# Winkelgeschwindigkeit als allgemeiner Ausdruck (nicht über Gelenk-Konfiguration)
# (fällt später sowieso weg)
omega_EE := Matrix(3,1,<omega_xEE; omega_yEE; omega_zEE>):
# Differenz als kinematische Zwangsbedingung aufstellen
h_rotg := Matrix(omega_W_i(1..3,LIJAC+1)) - omega_EE:
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
save b_rotg, sprintf("../codeexport/%s/tmp/jacobig_rot_%d_maple.m", robot_name, LIJAC):
# Ausdruck nochmal laden.
read sprintf("../codeexport/%s/tmp/jacobig_rot_%d_maple.m", robot_name, LIJAC):
b_rotg := b_rotg:
if codegen_act and codegen_debug then
  for i from 1 to 3 do
    for j from 1 to NQJ do
      MatlabExport(b_rotg(i,j), sprintf("../codeexport/%s/tmp/jacobig_rot_%d_floatb_%s_%d_%d_matlab.m", robot_name, LIJAC, base_method_name, i, j), codegen_opt):
    end do:
  end do:
end if:
if codegen_act then
  MatlabExport(b_rotg, sprintf("../codeexport/%s/tmp/jacobig_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
end if:
# Jacobi-Matrix der Rotationsmatrix (Rotatorisch)
# Ableitung der Rotationsmatrix nach den Gelenkwinkel (kann z.B. für die inverse Kinematik benutzt werden)
# Siehe [SchapplerTapOrt2019], Gl. 31, Term III und Gl. 27
R_0_i_s := convert_t_s( Trf_c(1 .. 3, 1..3, LIJAC+1) ):
Rb_0_i_s :=Reshape(R_0_i_s, 9,1):
J_Rb_s := Matrix(9, NQJ):
for i from 1 to 9 do
  for j from 1 to NQJ do
    J_Rb_s(i,j) := diff(Rb_0_i_s(i,1), qJ_s(j,1)):
  end do:
end do:
save J_Rb_s, sprintf("../codeexport/%s/tmp/jacobiR_rot_%d_floatb_%s_maple.m", robot_name, LIJAC, base_method_name):
if codegen_act and codegen_jacobi > 1 then
  MatlabExport(J_Rb_s, sprintf("../codeexport/%s/tmp/jacobiR_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
end if:
# Jacobi-Zeitableitungen
# Die Jacobi-Zeitableitungen werden z.B. zur Umrechnung zwischen EE-Beschleunigungen und Gelenk-Beschleunigungen benötigt.
JD_rotg_t := diff~(convert_s_t(b_rotg), t):
JD_rotg_s := convert_t_s(JD_rotg_t):
JD_rota_t := diff~(convert_s_t(b_rota), t):
JD_rota_s := convert_t_s(JD_rota_t):
JD_transla_t := diff~(convert_s_t(b_transl), t):
JD_transla_s := convert_t_s(JD_transla_t):
# Zeitableitung der Ableitung der Rotationsmatrix nach den Gelenkwinkeln. Dieser Term wird zur Berechnung der Zeitableitung der kinematischen Zwangsbedingungen für PKM benötigt.
# Das wird bei der Berechnung der Coriolis-Terme aus dieser Darstellung benötigt.
JD_Rb_t := diff~(convert_s_t(J_Rb_s), t):
JD_Rb_s := convert_t_s(JD_Rb_t):
# Matlab-Export
if codegen_act and codegen_jacobi > 2 then
  MatlabExport(JD_rotg_s, sprintf("../codeexport/%s/tmp/jacobigD_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
  interface(warnlevel=0): # Unterdrücke die Warnung für arctan
  MatlabExport(JD_rota_s, sprintf("../codeexport/%s/tmp/jacobiaD_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
  interface(warnlevel=3):
  MatlabExport(JD_transla_s, sprintf("../codeexport/%s/tmp/jacobiaD_transl_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
  MatlabExport(JD_Rb_s, sprintf("../codeexport/%s/tmp/jacobiRD_rot_%d_floatb_%s_matlab.m", robot_name, LIJAC, base_method_name), codegen_opt):
end if:


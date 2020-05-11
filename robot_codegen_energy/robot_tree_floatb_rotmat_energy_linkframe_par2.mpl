
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
read "../helper/proc_simplify2":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name, base_method_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
kin_constraints_exist := kin_constraints_exist: # Laden zur Einstellung der Term-Vereinfachung
;
# Einstellungen für Term-Vereinfachungen: 0=keine, 2=E_kin (einzelne), 3=auch E_kin (komplett)
if not assigned(simplify_options) or simplify_options(6)=-1 then
  # Die Vereinfachung ist ein Kompromiss aus Laufzeit des Programms und Güte der Ergebnisse
  # Bei zu komplizierten Termen stürzt Maple bei der Vereinfachung unvorhersehbar ab.
  # Die folgenden Regeln sind konservative Einstellungen, damit die Code-Generierung nicht zu lange dauert.
  if not kin_constraints_exist then # ohne Zwangsbedingungen. Annahme: Bringt dann sowieso nicht so viel
    use_simplify := 0: # Keine Vereinfachungen
  else # für Systeme mit Zwangsbedingungen
    if NJ <= 7 then
      use_simplify := 3: # standardmäßig bei einfachen Systemen alle simplify-Befehle anwenden
    else
      use_simplify := 0: # Sonst keine. Optimierung der einzelnen kinetischen Energien bringt sowieso nichts.
    end if:
  end if:
else
  use_simplify := simplify_options(6): # sechster Eintrag ist für Energie
end if:

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
  printf("%s. Kinetische Energie für Körper %d berechnet (im Körper-KS, mit Parametersatz 2).\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), i-1):#0=Basis
  # Terme vereinfachen (nur einzelne Energien und nicht die Summe).
  # Annahme: Zusammenfassung verschiedener Körper nicht zielführend
  if use_simplify>=2 then
    tmp_t1:=time(): tmp_l1 := length(T_b[i,1]):
    printf("%s. Beginne Vereinfachung für kinetische Energie (Param. 2; Körper %d). Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1):
    T_b[i,1] := simplify2(T_b[i,1]):
    tmp_t2:=time(): tmp_l2 := length(T_b[i,1]): 
    printf("%s. Terme für kinetische Energie (Param. 2) vereinfacht (Körper %d). Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
  T := T+T_b[i, 1]:

  # Terme vereinfachen (optional, Standardmäßig nur kinetische Energie einzelner Körper optimieren)
  # Für komplizierte Systeme hängt sich Maple hier auf.
  if use_simplify>=3 then
    tmp_t1:=time(): tmp_l1 := length(T):
    printf("%s. Beginne Vereinfachung für kinetische Energie (Param. 2; Summe bis Körper %d). Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1):
    T := simplify2(T):
    tmp_t2:=time(): tmp_l2 := length(T):
    printf("%s. Terme für kinetische Energie (Param. 2) vereinfacht (Körper %d). Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
end do:
# Maple Export
save T, sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
# Potential Energy
# Berechnung in Welt-KS ist bereits erfolgreich. Wird hier nicht nochmal durchgeführt.
# siehe robot_tree_floatb_twist_rotmat_energy_worldframe_par2.mw
# Export
printf("%s: Kinetische Energie (Param. 2) berechnet und gespeichert. Beginne Matlab-Export.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S")):
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


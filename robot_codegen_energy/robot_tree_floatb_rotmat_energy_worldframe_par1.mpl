
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
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
kin_constraints_exist := kin_constraints_exist: # Laden zur Einstellung der Term-Vereinfachung
;
# Einstellungen für Term-Vereinfachungen: 0=keine, 1=E_pot, 2=auch E_kin (einzelne), 3=auch E_kin (komplett)
if not assigned(simplify_options) or simplify_options(6)=-1 then
  # Die Vereinfachung ist ein Kompromiss aus Laufzeit des Programms und Güte der Ergebnisse
  # Bei zu komplizierten Termen stürzt Maple bei der Vereinfachung unvorhersehbar ab.
  # Die folgenden Regeln sind konservative Einstellungen, damit die Code-Generierung nicht zu lange dauert.
  if not kin_constraints_exist then # ohne Zwangsbedingungen. Annahme: Bringt dann sowieso nicht so viel
    use_simplify := 0:
  else # für Systeme mit Zwangsbedingungen
    if NJ <= 7 then
      use_simplify := 3: # standardmäßig bei einfachen Systemen alle simplify-Befehle anwenden
    elif NJ <= 11 then # das ist nur eine grobe Abschätzung
      use_simplify := 2: # Simplify reduzieren (Annahme: Optimierung Gesamt-Energie dauer länger)
    else
      use_simplify := 1: # simplify-Befehle weiter begrenzen (sonst hängt Maple sich auf)
    end if:
  end if:
else
  use_simplify := simplify_options(6): # sechster Eintrag ist für Energie
end if:

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
  printf("%s. Potentielle Energie aus Gravitation für Körper %d berechnet (im Welt-KS, Parametersatz 1).\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), i-1): #0=Basis
  # Terme vereinfachen (nur einzelne Energien und nicht die Summe.
  # Annahme: Zusammenfassung verschiedener Körper nicht zielführend
  if use_simplify>=1 then
    tmp_t1:=time(): tmp_l1 := length(U_b[i,1]):
    printf("%s. Beginne Vereinfachung für potentielle Energie (Param. 1; Körper %d). Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1):
    U_b[i,1] := simplify2(U_b[i,1]):
    tmp_t2:=time(): tmp_l2 := length(U_b[i,1]):
    printf("%s. Terme für potentielle Energie (Param. 1) vereinfacht (Körper %d). Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
  U_grav := U_grav+U_b[i, 1]:
  # Terme vereinfachen
  if use_simplify>=1 then
    tmp_t1:=time(): tmp_l1 := length(U_grav):
    printf("%s. Beginne Vereinfachung für potentielle Energie (Param. 1; Summe bis Körper %d). Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1):
    U_grav := simplify2(U_grav):
    tmp_t2:=time(): tmp_l2 := length(U_grav):
    printf("%s. Terme für potentielle Energie vereinfacht (Param. 1; Summe bis Körper %d). Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
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
  printf("%s. Kinetische Energie für Körper %d berechnet (im Welt-KS, Parametersatz 1)\n", FormatTime("%Y-%m-%d %H:%M:%S"), i-1):#0=Basis
  # Terme vereinfachen (nur einzelne Energien und nicht die Summe.
  # Annahme: Zusammenfassung verschiedener Körper nicht zielführend
  if use_simplify>=2 then
    tmp_t1:=time(): tmp_l1 := length(T_b[i,1]):
    printf("%s. Beginne Vereinfachung für kinetische Energie (Param. 1; Körper %d). Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1):
    T_b[i,1] := simplify2(T_b[i,1]):
    tmp_t2:=time(): tmp_l2 := length(T_b[i,1]):
    printf("%s. Terme für kinetische Energie (Param. 1) vereinfacht (Körper %d). Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
  T := T+T_b[i, 1]:

  # Terme vereinfachen (optional, Standardmäßig nur kinetische Energie einzelner Körper optimieren)
  # Für komplizierte Systeme hängt sich Maple hier auf.
  if use_simplify>=3 then
    tmp_t1:=time(): tmp_l1 := length(T):
    printf("%s. Beginne Vereinfachung für kinetische Energie (Param. 1; Summe bis Körper %d). Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1):
    T := simplify2(T):
    tmp_t2:=time(): tmp_l2 := length(T):
    printf("%s. Terme für kinetische Energie (Param. 1; Summe bis Körper %d) vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i-1, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
end do:
# Maple Export
save T, sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
# Matlab Export
printf("%s: Energie (Param. 1) berechnet und gespeichert. Beginne Matlab-Export.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S")):
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
  MatlabExport(U_s_fixb, sprintf("../codeexport/%s/tmp/energy_potential_fixb_worldframe_par1_matlab.m", robot_name), codegen_opt):
end if:
# Maple Export (Fixed-Base)
save U_s_fixb, sprintf("../codeexport/%s/tmp/energy_potential_fixb_worldframe_par1_maple.m", robot_name):
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
# Maple Export (Fixed-Base)
save T_s_fixb, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_worldframe_par1_maple.m", robot_name):



# Center of Mass Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Schwerpunkts-Kinematik (Positionen)
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
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
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
printf("%s. Generiere Schwerpunktskinematik für %s\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):  
kin_constraints_exist := kin_constraints_exist: # nur zum Abschätzen der Komplexität
;
if not assigned(simplify_options) or simplify_options(3)=-1 then # Standard-Einstellungen:
  if not kin_constraints_exist then # normale serielle Ketten und Baumstrukturen
    use_simplify := 0: # Standardmäßig aus
  else # mit kinematischen Zwangsbedingungen
    use_simplify := 1: # standardmäßig simplify-Befehle anwenden
  end if:
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(3): # dritter Eintrag ist für CoM-Kinematik
end if:

# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
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
  if use_simplify>=1 then
    tmp_t11:=time():
    tmp_l11 := length(r_W_i_Si(1 .. 3, i)):
    printf("%s. Vereinfache Schwerpunktskoordinaten. Länge: %d (r_i_i_Si).\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l11):
    r_W_i_Si(1 .. 3, i) := simplify2(r_W_i_Si(1 .. 3, i)):
    tmp_l21 := length(r_W_i_Si(1 .. 3, i)):
    tmp_t21:=time():
  end if:
  mr_W_i_Si(1 .. 3, i) := Multiply(Matrix(Trf_c(1 .. 3, 1 .. 3, i)), mr_i_i_Si(1 .. 3, i)):
  r_W_W_Si(1 .. 3, i) :=  Matrix(Trf_c(1 .. 3, 4, i)) + Matrix(r_W_i_Si(1 .. 3, i)):
  printf("%s. Schwerpunktsposition in Weltkoordinaten für Körper %d aufgestellt.\n", FormatTime("%Y-%m-%d %H:%M:%S"), i-1):#0=Basis
  if use_simplify>=1 then
    tmp_t12:=time():
    tmp_l12 := length(mr_W_i_Si(1 .. 3, i)):
    tmp_l13 := length(r_W_W_Si(1 .. 3, i)):
    printf("%s. Vereinfache Schwerpunktskoordinaten. Länge: %d/%d (r_W_W_Si/mr_W_i_Si).\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l13, tmp_l12):
    mr_W_i_Si(1 .. 3, i) := simplify2(mr_W_i_Si(1 .. 3, i)):
    r_W_W_Si(1 .. 3, i) := simplify2(r_W_W_Si(1 .. 3, i)):
    mr_W_W_Si(1 .. 3, i) := simplify2(mr_W_W_Si(1 .. 3, i)):
    tmp_l22 := length(mr_W_i_Si(1 .. 3, i)):
    tmp_l23 := length(r_W_W_Si(1 .. 3, i)):
    tmp_t22:=time():
    printf("%s. Terme für Schwerpunktskoord. vereinfacht. Länge: %d->%d / %d->%d / %d->%d (r_W_i_Si / mr_W_i_Si / r_W_W_Si). Rechenzeit %1.1fs und %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l11, tmp_l21, tmp_l12, tmp_l22, tmp_l13, tmp_l23, tmp_t21-tmp_t11, tmp_t22-tmp_t12):
  end if:
end do:
# Maple Export
save mr_W_i_Si, r_W_W_Si, r_W_i_Si, sprintf("../codeexport/%s/tmp/kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
printf("Maple-Ausdrücke exportiert.\n"):
# Matlab-Export
if codegen_act then
  MatlabExport(convert_t_s(r_W_W_Si), sprintf("../codeexport/%s/tmp/fkine_com_floatb_%s_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Calculate CoM of the whole robot
# Berechne den Gesamt-Schwerpunkt mit den beiden Parametersätzen. Ist etwas redundant (und widersprüchlich zur par1-Benennung des Skripts).
# Variable mit par2 wird später noch benötigt (Impuls für ZMP)
for codegen_dynpar from 1 to 2 do
  c:=Matrix(3,1):
  M_ges:=0:
  for i from 1 to NL do
    if codegen_dynpar = 1 then
      c:=c + Matrix(r_W_W_Si(1 .. 3,i)*M(i,1)):
    else
    	 # Formel: M * r_W_W_Si = M * (r_W_W_i + r_W_i_Si)
      c:=c + Matrix(Trf_c(1 .. 3, 4, i))*M(i,1) + Matrix(mr_W_i_Si(1 .. 3,i)):
    end if:
    M_ges:=M_ges + M[i,1]:
  end do:
  c:=c/M_ges:
  # Maple-Export
  save c, sprintf("../codeexport/%s/tmp/kinematics_com_total_worldframe_floatb_%s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
  # Matlab-Export
  if codegen_act then
    MatlabExport(convert_t_s(c), sprintf("../codeexport/%s/tmp/com_total_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
  end if:
end do:


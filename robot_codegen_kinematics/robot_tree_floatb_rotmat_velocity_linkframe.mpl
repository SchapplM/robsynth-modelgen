
# Velocity Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Geschwindigkeit von Koordinatensystemen und Schwerpunkten
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb_twist -> floating base wird durch base twist (Geschwindigkeit der Basis) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# velocity -> Berechnung der Geschwindigkeit aller Segmente
# linkframe -> Berechnung der Geschwindigkeit im Körper-KS (KSi)
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
printf("%s. Generiere Geschwindigkeit für %s (Herleitung im Körper-KS)\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):  
kin_constraints_exist := kin_constraints_exist: # nur zum Abschätzen der Komplexität
;
# Term-Vereinfachungen einstellen
if not assigned(simplify_options) or simplify_options(4)=-1 then # Standard-Einstellungen:
  if not kin_constraints_exist then # normale serielle Ketten und Baumstrukturen
    use_simplify := 0: # Standardmäßig aus
  else # mit kinematischen Zwangsbedingungen
    use_simplify := 1: # standardmäßig simplify-Befehle anwenden
  end if:
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(4): # vierter Eintrag ist für Geschwindigkeit
end if:

# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Zeitableitungen der MDH-Drehwinkel laden.
# Die Berechnung soll nur an einer Stelle erfolgen. Siehe robot_tree_velocity_mdh_angles.mw.
read sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):
thetaD := thetaD:

# Calculate Velocities
# First assume fixed base model with base velocity and acceleration set to zero
# Anfangsgeschwindigkeit definieren für floating base model
# Velocities of Frames
# 
# Analog zu [Ortmaier2014], Gl. (7.6)
# Geschwindigkeit der Basis der Koordinatensysteme der Körper (ausgedrückt im Körper-KS)
rD_i_i := Matrix(3, NL):
# Winkelgeschwindigkeit von Körper i ausgedrückt im Körper-KS
omega_i_i := Matrix(3, NL):
# Anfangsgeschwindigkeit der Basis:
# twist: Basis-Geschwindigkeit bzgl Welt-KS ausgedrückt im Basis-KS
# eulxyz: V_base_t beinhaltet die Geschwindigkeit der Basis im Welt-KS, ausgedrückt im Welt-KS. Daher ist für eine Darstellung im Körper-KS noch die Rotation erforderlich.
if base_method_name = "twist" then:
  rD_i_i(1..3,1) := V_base_t(1..3,1):
  omega_i_i(1..3,1) := V_base_t(4..6,1):
end:
if base_method_name = "eulxyz" then:
  rD_i_i(1..3,1) := Transpose(Trf_c(1..3, 1..3, 1)) . V_base_t(1..3,1):
  omega_i_i(1..3,1) := Transpose(Trf_c(1..3, 1..3, 1)) . T_basevel . V_base_t(4..6,1):
end:
# Erhöhe die Geschwindigkeit jedes Körpers
# Betrachte dazu nur die durch Gelenke angetriebenen Körper, nicht die Basis
for i from 1 to NJ do # Gelenke durchgehen
  # Körper der von Gelenkwinkel i bewegt wird: Körperindex i+1
  # Vorgängerkörper bestimmen
  j := v(i) + 1:
  # Geschwindigkeit des Vorgängers; Trf_c(...,i+1) enthält z-Achse des Körperkoordinatensystems, das von qi bewegt wird.
  # [Ortmaier2014] (7.7) (S.115) (dort falsche Indizes für MDH), [KhalilDombre2002] (9.14)
  R_j_i := Trf(1..3,1..3,i): # Rotation vom Vorgänger-Körper (j) zu diesem Körper (i+1)
  R_i_j := Transpose(R_j_i):
  # [GautierKhalil1988], equ.7: omega_jj aus [GautierKhalil1988] entspricht omega_i_i(1 .. 3, i+1) hier
  if sigma(i) = 0 then # Drehgelenk
    omega_i_i(1 .. 3, i+1) := Multiply(R_i_j,Matrix(3,1,omega_i_i(1 .. 3, j))) + thetaD(i,1)*<0;0;1>:
  else: # Schubgelenk
    omega_i_i(1 .. 3, i+1) := Multiply(R_i_j,Matrix(3,1,omega_i_i(1 .. 3, j)))
  end if:
  # Terme vereinfachen (Teil 1)
  if use_simplify>=1 then
    tmp_t11:=time():
    tmp_l11 := length(omega_i_i(1 .. 3, i+1)):
    omega_i_i(1 .. 3, i+1) := simplify2(omega_i_i(1 .. 3, i+1)):
    tmp_l21 := length(omega_i_i(1 .. 3, i+1)):
    tmp_t21:=time():
  end if:
  # Vektor vom Ursprung des vorherigen Koordinatensystems zu diesem KS
  r_j_j_i := Trf(1 .. 3, 4, i):
  # [GautierKhalil1988], equ.8: v_jj aus [GautierKhalil1988] entspricht rD_i_i(1 .. 3, i+1) hier
  if sigma(i) = 0 then # Drehgelenk
    rD_i_i(1 .. 3, i+1) := Multiply( R_i_j, ( rD_i_i(1 .. 3, j) + CrossProduct(omega_i_i(1 .. 3, j), r_j_j_i) ) ):
  else: # Schubgelenk
    rD_i_i(1 .. 3, i+1) := Matrix(Multiply( R_i_j, ( rD_i_i(1 .. 3, j) + CrossProduct(omega_i_i(1 .. 3, j), r_j_j_i) ) ) ) + Matrix(dD(i,1)*<0;0;1>):
  end if:
  printf("%s. Geschwindigkeit für Körperkoordinatensystem %d aufgestellt (Herleitung im Körper-KS).\n", \
     FormatTime("%Y-%m-%d %H:%M:%S"), i-1): #0=Basis
  # Terme vereinfachen (Teil 2)
  if use_simplify>=1 then
    tmp_t12:=time():
    tmp_l12 := length(rD_i_i(1 .. 3, i+1)):
    rD_i_i(1 .. 3, i+1) := simplify2(rD_i_i(1 .. 3, i+1)):
    tmp_l22 := length(rD_i_i(1 .. 3, i+1)):
    tmp_t22:=time():
    printf("%s: Terme für Geschwindigkeiten vereinfacht. Länge: %d->%d / %d->%d. Rechenzeit %1.1fs und %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l11, tmp_l21, tmp_l12, tmp_l22, tmp_t21-tmp_t11, tmp_t22-tmp_t12):
  end if:
end do:
# Export
# Maple Export
save omega_i_i, rD_i_i, sprintf("../codeexport/%s/tmp/velocity_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
printf("%s. Maple-Ausdrücke exportiert.\n", FormatTime("%Y-%m-%d %H:%M:%S")):

# Matlab Export
if codegen_act then
  MatlabExport(convert_t_s(omega_i_i), sprintf("../codeexport/%s/tmp/velocity_omegaii_floatb_%s_linkframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rD_i_i), sprintf("../codeexport/%s/tmp/velocity_rDii_floatb_%s_linkframe_matlab.m", robot_name, base_method_name), codegen_opt):
  printf("%s. Geschwindigkeiten in Matlab exportiert.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:


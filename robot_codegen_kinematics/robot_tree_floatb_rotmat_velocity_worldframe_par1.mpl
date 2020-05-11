
# Velocity Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Geschwindigkeit von Koordinatensystemen und Schwerpunkten
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# velocity -> Berechnung der Geschwindigkeit aller Segmente
# worldframe -> Berechnung der Geschwindigkeit im Welt-KS (KSW)
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
printf("%s. Generiere Geschwindigkeit für %s (Herleitung im Welt-KS)\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name):
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
else
  use_simplify := simplify_options(4): # vierter Eintrag ist für Geschwindigkeit
end if:

# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
read sprintf("../codeexport/%s/tmp/kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
r_W_W_Si := r_W_W_Si:
r_W_i_Si := r_W_i_Si:
# Zeitableitungen der MDH-Drehwinkel laden.
# Die Berechnung soll nur an einer Stelle erfolgen. Siehe robot_tree_velocity_mdh_angles.mw.
read sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):
thetaD := thetaD:

# Calculate Velocities
# First assume fixed base model with base velocity and acceleration set to zero
# Anfangsgeschwindigkeit definieren für floating base model
# Velocities of Frames
# Anfangsgeschwindigkeit der Basis
# Analog zu [Ortmaier2014], Gl. (7.6)
# Geschwindigkeit der Basis der Koordinatensysteme der Körper
rD_W_i := Matrix(3, NL):
rD_W_i(1..3,1) := V_base_t(1..3,1):
# Winkelgeschwindigkeit von Körper i ausgedrückt im KS W
omega_W_i := Matrix(3, NL):
if base_method_name = "twist" then:
  omega_W_i(1..3,1) := V_base_t(4..6,1):
end:
if base_method_name = "eulxyz" then:
  omega_W_i(1..3,1) := T_basevel . V_base_t(4..6,1):
end:
printf("%s. Nutze die Methode %s für die Basis-Drehgeschwindigkeit\n", FormatTime("%Y-%m-%d %H:%M:%S"), base_method_name):
# Erhöhe die Geschwindigkeit jedes Körpers
# Betrachte dazu nur die durch Gelenke angetriebenen Körper, nicht die Basis
for i from 1 to NJ do # Gelenke durchgehen
  # Körper der von Gelenkwinkel i bewegt wird: Index i+1
  # Vorgängerkörper bestimmen
  j := v(i) + 1:
  if sigma(i) = 0 then # Drehgelenk
    # Geschwindigkeit des Vorgängers; Trf_c(...,i+1) enthält z-Achse des Körperkoordinatensystems, das von qi bewegt wird.
    # [Ortmaier2014] (7.7) (S.115) (dort falsche Indizes für MDH), [KhalilDombre2002] (9.14)
    omega_W_i(1 .. 3, i+1) := omega_W_i(1 .. 3, j) + thetaD(i,1)*Trf_c(1 .. 3, 3, i+1):
  else: # Schubgelenk
    # [Ortmaier2014] (7.12) (S.116)
    omega_W_i(1 .. 3, i+1) := omega_W_i(1 .. 3, j):
  end if:
  # Terme vereinfachen (Teil 1)
  if use_simplify=1 then
    tmp_t11:=time():
    tmp_l11 := length(omega_W_i(1 .. 3, i+1)):
    omega_W_i(1 .. 3, i+1) := simplify2(omega_W_i(1 .. 3, i+1)):
    tmp_l21 := length(omega_W_i(1 .. 3, i+1)):
    tmp_t21:=time():
  end if:
  # Vektor vom Ursprung des vorherigen Koordinatensystems zu diesem KS
  r_W_im1_i := -Trf_c(1 .. 3, 4, j) + Trf_c(1 .. 3, 4, i+1):
  if sigma(i) = 0 then # Drehgelenk
    # [Ortmaier2014] (7.10) (S.115), [KhalilDombre2002] (9.15)
    rD_W_i(1 .. 3, i+1) := rD_W_i(1 .. 3, j) + CrossProduct(omega_W_i(1 .. 3, j), r_W_im1_i):
  else: # Schubgelenk
    # [Ortmaier2014] (7.15) (S.116), [KhalilDombre2002] (9.15)
    rD_W_i(1 .. 3, i+1) := rD_W_i(1 .. 3, j) + CrossProduct(omega_W_i(1 .. 3, j), r_W_im1_i) + dD(i,1)*Trf_c(1 .. 3, 3, i+1):
  end if:
  printf("%s. Geschwindigkeit für Körperkoordinatensystem %d aufgestellt (Herleitung im Welt-KS).\n",\
    FormatTime("%Y-%m-%d %H:%M:%S"), i-1):#0=Basis
  # Terme vereinfachen (Teil 2)
  if use_simplify=1 then
    tmp_t12:=time():
    tmp_l12 := length(rD_W_i(1 .. 3, i+1)):
    rD_W_i(1 .. 3, i+1) := simplify2(rD_W_i(1 .. 3, i+1)):
    tmp_l22 := length(rD_W_i(1 .. 3, i+1)):
    tmp_t22:=time():
    printf("%s: Terme für Geschwindigkeiten vereinfacht. Länge: %d->%d / %d->%d. Rechenzeit %1.1fs und %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l11, tmp_l21, tmp_l12, tmp_l22, tmp_t21-tmp_t11, tmp_t22-tmp_t12):
  end if:
end do:
# Velocities of Center of Mass
# 
rD_W_Si := Matrix(3, NL):
for i to NL do
  # [Ortmaier2014] (7.17) (S.118), [KhalilDombre2002] (9.15)
  rD_W_Si(1 .. 3, i) := Matrix(rD_W_i(1 .. 3, i)) + Matrix(CrossProduct(omega_W_i(1 .. 3, i), r_W_i_Si(1 .. 3, i))):
  printf("%s. Geschwindigkeit für Körperschwerpunkt %d aufgestellt.\n", FormatTime("%Y-%m-%d %H:%M:%S"), i-1):#0=Basis
  if use_simplify=1 then
    tmp_t1:=time():
    tmp_l1 := length(rD_W_Si(1 .. 3, i)):
    rD_W_Si(1 .. 3, i) := simplify2(rD_W_Si(1 .. 3, i)):
    tmp_l2 := length(rD_W_Si(1 .. 3, i)):
    tmp_t2:=time():
    printf("%s: Terme für Schwerpunkts-Geschwindigkeiten vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
end do:

# Exportieren
# Basis-Position aus Termen entfernen. Die Basis-Position kann keinen Einfluss auf die Geschwindigkeiten der einzelnen Körper haben.
# Bei manchen komplexen Systemen scheint der Term aber trotzdem nicht zu verschwinden, da Maple die Vereinfachung nicht erkennt
for i from 1 to 3 do
  omega_W_i := subs({X_base_s[i,1]=0},omega_W_i):
  rD_W_i := subs({X_base_s[i,1]=0},rD_W_i):
  rD_W_Si := subs({X_base_s[i,1]=0},rD_W_Si):
end do:
# Maple Export
save omega_W_i, rD_W_i, rD_W_Si, sprintf("../codeexport/%s/tmp/velocity_worldframe_floatbase_%s_par1_maple.m", robot_name, base_method_name):
printf("%s. Maple-Ausdrücke exportiert.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Matlab Export
if codegen_act then
  MatlabExport(convert_t_s(omega_W_i), sprintf("../codeexport/%s/tmp/velocity_omegai0_floatb_%s_worldframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rD_W_i), sprintf("../codeexport/%s/tmp/velocity_rDi0_floatb_%s_worldframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rD_W_Si), sprintf("../codeexport/%s/tmp/velocity_rDSi0_floatb_%s_worldframe_par1_matlab.m", robot_name, base_method_name), codegen_opt):
  printf("Geschwindigkeiten in Matlab exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:



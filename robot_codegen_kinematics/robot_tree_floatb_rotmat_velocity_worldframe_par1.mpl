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
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# 
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I (WS 2014/15)
# [Ott2008] Cartesian Impedance Control of Redundant and Flexible-Joint Robots
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
printf("Generiere Geschwindigkeit für %s\n", robot_name):
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s_kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
read sprintf("../codeexport/%s_kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
r_W_W_Si := r_W_W_Si:
r_W_i_Si := r_W_i_Si:
# Lade Ausdrücke für kinematische Zwangsbedingungen (Verknüpfung von MDH-Gelenkwinkeln durch verallgemeinerte Koordinaten)
read sprintf("../codeexport/%s_kinematic_constraints_maple_inert.m", robot_name):
# Zeitableitung der Drehwinkel berechnen
# Ersetze die MDH-Winkel durch verallgemeinerte Koordinaten
# Falls die Gelenkwinkel nicht direkt mit verallgemeinerten Koordinaten überstimmen (bei Kopplungen, kinematischen Schleifen) steht hier eine längere Berechnung. Ansonsten reicht das triviale Einsetzen:
# thetaD := qJD_t:
# Ersetze die MDH-Winkel durch verallgemeinerte Koordinaten
theta_qt := theta:
for ii from 1 to NJ do
  for jj from 1 to RowDimension(kintmp_qt) do
    theta_qt(ii, 1) := subs( { kintmp_t(jj, 1) = kintmp_qt(jj, 1) }, theta_qt(ii, 1) ): 
  end do:
end do:
# Zeitableitung der MDH-Winkel in Abhängigkeit der verallgemeinerten Koordinaten
thetaD := Matrix(NJ, 1):
for i from 1 to NJ do
  thetaD(i,1) := diff(theta_qt(i,1), t):
end do:
# Ausdruck für Zeitableitungen der MDH-Winkel exportieren
if codegen_act then
  MatlabExport(convert_t_s(thetaD), sprintf("../codeexport/%s_velocity_mdh_angles_matlab.m", robot_name), codegen_opt):
end if:
# Ausdruck für Maple speichern
save thetaD, sprintf("../codeexport/%s_velocity_mdh_angles_maple.m", robot_name):
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
if base_method_name = "eulangrpy" then:
  omega_W_i(1..3,1) := T_basevel . V_base_t(4..6,1):
end:
printf("Nutze die Methode %s für die Basis-Drehgeschwindigkeit\n", base_method_name):

# Erhöhe die Geschwindigkeit jedes Körpers
# Betrachte dazu nur die durch Gelenke angetriebenen Körper, nicht die Basis
for i from 1 to NJ do # Gelenke durchgehen
  # Körper der von Gelenkwinkel i bewegt wird: Index i+1
  # Vorgängerkörper bestimmen
  j := v(i) + 1:
  # Geschwindigkeit des Vorgängers; Trf_c(...,i+1) enthält z-Achse des Körperkoordinatensystems, das von qi bewegt wird.
  # [Ortmaier2014] (7.7) (S.115) (dort falsche Indizes für MDH), [KhalilDombre2002] (9.14)
  omega_W_i(1 .. 3, i+1) := omega_W_i(1 .. 3, j) + thetaD(i,1)*Trf_c(1 .. 3, 3, i+1):
  # Vektor vom Ursprung des vorherigen Koordinatensystems zu diesem KS
  r_W_im1_i := -Trf_c(1 .. 3, 4, j) + Trf_c(1 .. 3, 4, i+1):
  # [Ortmaier2014] (7.9) (S.115), [KhalilDombre2002] (9.15)
  rD_W_i(1 .. 3, i+1) := rD_W_i(1 .. 3, j) + CrossProduct(omega_W_i(1 .. 3, j), r_W_im1_i):
  printf("Geschwindigkeit für Körperkoordinatensystem %d aufgestellt. %s\n", i, FormatTime("%Y-%m-%d %H:%M:%S")):
end do:
# Velocities of Center of Mass
rD_W_Si := Matrix(3, NL):
for i to NL do
  # [Ortmaier2014] (7.17) (S.118), [KhalilDombre2002] (9.15)
  rD_W_Si(1 .. 3, i) := Matrix(rD_W_i(1 .. 3, i)) + Matrix(CrossProduct(omega_W_i(1 .. 3, i), r_W_i_Si(1 .. 3, i))):
  printf("Geschwindigkeit für Körperschwerpunkt %d aufgestellt. %s\n", i, FormatTime("%Y-%m-%d %H:%M:%S")):
end do:
# Maple Export
save omega_W_i, rD_W_i, rD_W_Si, sprintf("../codeexport/%s_velocity_worldframe_floatbase_%s_par1_maple", robot_name, base_method_name):
save omega_W_i, rD_W_i, rD_W_Si, sprintf("../codeexport/%s_velocity_worldframe_floatbase_%s_par1_maple.m", robot_name, base_method_name):
printf("Maple-Ausdrücke exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Matlab Export
if codegen_act then
  MatlabExport(convert_t_s(omega_W_i), sprintf("../codeexport/%s_velocity_omegai0_floatb_%s_worldframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rD_W_i), sprintf("../codeexport/%s_velocity_rDi0_floatb_%s_worldframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rD_W_Si), sprintf("../codeexport/%s_velocity_rDSi0_floatb_%s_worldframe_par1_matlab.m", robot_name, base_method_name), codegen_opt):
  printf("Geschwindigkeiten in Matlab exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:


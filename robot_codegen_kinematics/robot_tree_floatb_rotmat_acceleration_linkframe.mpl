
# Acceleration Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Beschleunigung von Koordinatensystemen und Schwerpunkten
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb_twist -> floating base wird durch base twist (Geschwindigkeit der Basis) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# acceleration -> Berechnung der Beschleunigung aller Segmente
# linkframe -> Berechnung der Geschwindigkeit im Körper-KS (KSi)

# Sources
# [KhalilDombre2002] Modeling, Identification and Control of Robots
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
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
printf("Generiere Beschleunigung für %s (Herleitung im Körper-KS)\n", robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Ergebnisse der Geschwindigkeit laden
read sprintf("../codeexport/%s/tmp/velocity_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
omega_i_i:=omega_i_i:
rD_i_i:=rD_i_i:
# Zeitableitungen der MDH-Drehwinkel laden.
# Die Berechnung soll nur an einer Stelle erfolgen. Siehe robot_tree_velocity_mdh_angles.mw.
read sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):
thetaD := thetaD:
dD :=dD:
# Zeitableitungen der Geschwindigkeit laden.
# Die Berechnung soll nur an einer Stelle erfolgen. Siehe robot_tree_acceleration_mdh_angles.mw.
read sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_maple.m", robot_name):
thetaDD :=thetaDD:
dDD := dDD:
# Calculate Acceleration
# First assume fixed base model with base velocity and acceleration set to zero
# Anfangsgeschwindigkeit definieren für floating base model
# Acceleration of Frames
# 
# Beschleunigung der Basis der Koordinatensysteme der Körper (ausgedrückt im Körper-KS)
rDD_i_i := Matrix(3, NL):
# Winkelbeschleunigung von Körper i ausgedrückt im Körper-KS
omegaD_i_i := Matrix(3, NL):
# Anfangsgeschwindigkeit der Basis:
# twist: Basis-Geschwindigkeit bzgl Welt-KS ausgedrückt im Basis-KS
# eulangrpy: V_base_t beinhaltet die Geschwindigkeit der Basis im Welt-KS, ausgedrückt im Welt-KS. Daher ist für eine Darstellung im Körper-KS noch die Rotation erforderlich.
if base_method_name = "twist" then:
  rDD_i_i(1..3,1) := diff~(V_base_t(1..3,1),t):
  omegaD_i_i(1..3,1) := diff~(V_base_t(4..6,1),t):
end:
if base_method_name = "eulangrpy" then:
  rDD_i_i(1..3,1) := Transpose(Trf_c(1..3, 1..3, 1)) . diff~(V_base_t(1..3,1),t):
  omegaD_i_i(1..3,1) := Transpose(Trf_c(1..3, 1..3, 1)) . T_basevel . diff~(V_base_t(4..6,1),t):
end:
# Erhöhe die Beschleunigung jedes Körpers
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
  # [Ortmaier](7.14) (S.99) & (7.19) (S.99)
  if sigma(i) = 0 then # Drehgelenk
    omegaD_i_i(1 .. 3, i+1) := Matrix(Multiply(R_i_j,( Matrix(3,1,omegaD_i_i(1 .. 3, j))))) + Matrix( thetaDD(i,1)*<0;0;1>) + Matrix(thetaD(i,1)* CrossProduct(Matrix(3,1,omega_i_i(1 .. 3, j)),<0;0;1>)) :
  else: # Schubgelenk
    omegaD_i_i(1 .. 3, i+1) := Multiply(R_i_j, Matrix(3,1,omegaD_i_i(1 .. 3, j))): 
  end if:
  
  # Vektor vom Ursprung des vorherigen Koordinatensystems zu diesem KS
  r_j_j_i := Trf(1 .. 3, 4, i):
  # [GautierKhalil1988], equ.8: v_jj aus [GautierKhalil1988] entspricht rD_i_i(1 .. 3, i+1) hier
  # [Ortmaier](7.17) (S.99) & (7.22) (S.100)
  if sigma(i) = 0 then # Drehgelenk
    rDD_i_i(1 .. 3, i+1) := Multiply( R_i_j, ( rDD_i_i(1 .. 3, j) )+ omegaD_i_i(1 .. 3, j) &x r_j_j_i + CrossProduct(omega_i_i(1..3,j),(omega_i_i(1..3,j) &x r_j_j_i ))) :
  else: # Schubgelenk
    rDD_i_i(1 .. 3, i+1) := Matrix( Multiply( R_i_j, ( rDD_i_i(1 .. 3, j)  + CrossProduct(omega_i_i(1..3,j), (omega_i_i(1..3,j) &x r_j_j_i )) + CrossProduct(omegaD_i_i(1..3,j), r_j_j_i)))) + Matrix( 2* dD(i,1) * CrossProduct(omega_i_i(1..3,j),<0;0;1>)) + Matrix(dDD(i,1)*<0;0;1>)  : 
  end if:
  printf("Beschleunigung für Körperkoordinatensystem %d aufgestellt (Herleitung im Körper-KS). %s\n", i, FormatTime("%Y-%m-%d %H:%M:%S")): #0=Basis
end do:

# Acceleration of Center of Mass
NULL;
rDD_i_Si := Matrix(3, NL):

for i to NL do 
  rDD_i_Si(1 .. 3, i) := Matrix(rDD_i_i(1 .. 3, i))+Matrix(CrossProduct(omegaD_i_i(1 .. 3, i), r_i_i_Si(1 .. 3, i)))+Matrix(CrossProduct(omega_i_i(1 .. 3, i), CrossProduct(omega_i_i(1 .. 3, i), r_i_i_Si(1 .. 3, i)))): 
  printf("Beschleunigung für Körperschwerpunkt %d aufgestellt. %s\n", i-1, FormatTime("%Y-%m-%d %H:%M:%S")) 
end do:
# Export
# Maple Export
save omegaD_i_i, rDD_i_i,rDD_i_Si , sprintf("../codeexport/%s/tmp/acceleration_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
printf("Maple-Ausdrücke exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):

# Matlab Export

if codegen_act then
  MatlabExport(convert_t_s(omegaD_i_i), sprintf("../codeexport/%s/tmp/acceleration_omegaDii_floatb_%s_linkframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rDD_i_i), sprintf("../codeexport/%s/tmp/acceleration_rDDii_floatb_%s_linkframe_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(convert_t_s(rDD_i_Si), sprintf("../codeexport/%s/tmp/acceleration_rDDiSi_floatb_%s_linkframe_matlab.m", robot_name, base_method_name), codegen_opt):
  printf("Beschleunigung in Matlab exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:


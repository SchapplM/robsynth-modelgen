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
read "../robot_codegen_definitions/robot_env":
printf("Generiere Geschwindigkeit für %s\n", robot_name):
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
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

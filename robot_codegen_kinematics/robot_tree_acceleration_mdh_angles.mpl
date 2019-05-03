
# Acceleration Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Beschleunigung von Koordinatensystemen und Schwerpunkten
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# acceleration_mdh_angles  -> Berechnung der Beschleunigung der MDH-Koordinaten (Drehung und Verschiebung in z-Richtung. Diese können zeitabhängig sein.)
# Sources
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I (WS 2014/15)
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
read "../robot_codegen_definitions/robot_env":
printf("Generiere Beschleunigung für %s\n", robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Lade Ausdrücke für kinematische Zwangsbedingungen (Verknüpfung von MDH-Gelenkwinkeln durch verallgemeinerte Koordinaten)
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
# Ergebnisse der Geschwindigkeit laden
read sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):
thetaD:= thetaD:
dD:=dD:
# Zeitableitung der Drehwinkelgeschwindigkeit berechnen
# Ersetze die MDH-Winkel durch verallgemeinerte Koordinaten
# Falls die Gelenkwinkel nicht direkt mit verallgemeinerten Koordinaten überstimmen (bei Kopplungen, kinematischen Schleifen) steht hier eine längere Berechnung. Ansonsten reicht das triviale Einsetzen:
# thetaDD := qJDD_t:
# Ersetze die MDH-Drehung und Verschiebung entlang der z-Achse durch verallgemeinerte Koordinaten
theta_qt := theta:
d_qt := d:
for ii from 1 to NJ do
  for jj from 1 to RowDimension(kintmp_qt) do
    theta_qt(ii, 1) := subs( { kintmp_t(jj, 1) = kintmp_qt(jj, 1) }, theta_qt(ii, 1) ):
    d_qt(ii, 1)     := subs( { kintmp_t(jj, 1) = kintmp_qt(jj, 1) }, d_qt(ii, 1) ): 
  end do:
end do:
# Zeitableitung der MDH-Drehung und Verschiebung entlang der z-Achse in Abhängigkeit der verallgemeinerten Koordinaten
thetaD := Matrix(NJ, 1):
dD := Matrix(NJ, 1):
for i from 1 to NJ do
  thetaD(i,1) := diff(theta_qt(i,1), t):
  dD(i,1)     := diff(d_qt(i,1), t):
end do:
# Zeitableitung der Geschwindigkeit in Abhängigkeit der verallgemeinerten Koordinaten 
thetaD_qt := thetaD:
dD_qt := dD:
thetaDD := Matrix(NJ, 1):
dDD:= Matrix(NJ, 1):
for i from 1 to NJ do:
  thetaDD(i,1):= diff(thetaD_qt(i,1),t):
  dDD(i,1)     := diff(dD_qt(i,1),t):
end do:
# Ausdruck für Zeitableitungen der Geschwindigkeit exportieren
if codegen_act then
  MatlabExport(convert_t_s(thetaDD), sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_matlab.m", robot_name), codegen_opt):
  MatlabExport(convert_t_s(dDD), sprintf("../codeexport/%s/tmp/acceleration_mdh_deltaz_matlab.m", robot_name), codegen_opt):
end if:
# Ausdruck für Maple speichern
save thetaDD, dDD, sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_maple.m", robot_name):
save thetaDD, dDD, sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_maple", robot_name):


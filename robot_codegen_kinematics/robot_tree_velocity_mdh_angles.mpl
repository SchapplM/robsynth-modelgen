
# Velocity Calculation for the Robot based on MDH frames
# Introduction
# Berechnung der Geschwindigkeit von Koordinatensystemen und Schwerpunkten
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# velocity_mdh_angles  -> Berechnung der Geschwindigkeit der MDH-Koordinaten (Drehung und Verschiebung in z-Richtung. Diese können zeitabhängig sein.
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
read "../robot_codegen_definitions/robot_env":
printf("%s. Generiere Geschwindigkeit für %s\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Lade Ausdrücke für kinematische Zwangsbedingungen (Verknüpfung von MDH-Gelenkwinkeln durch verallgemeinerte Koordinaten)
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
kintmp_qt := kintmp_qt:
kintmp_qt := kintmp_qt:
kin_constraints_exist := kin_constraints_exist:
kintmp_subsexp := kintmp_subsexp:
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

# Zeitableitung der Drehwinkel berechnen
# Ersetze die MDH-Winkel durch verallgemeinerte Koordinaten
# Falls die Gelenkwinkel nicht direkt mit verallgemeinerten Koordinaten überstimmen (bei Kopplungen, kinematischen Schleifen) steht hier eine längere Berechnung. Ansonsten reicht das triviale Einsetzen:
# thetaD := qJD_t:
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
if kin_constraints_exist then # ist nur rechenaufwändig, wenn ZB vorliegen
  printf("%s. Zeitableitung der MDH-Winkel gebildet.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:
# Terme vereinfachen

if use_simplify>=1 and kin_constraints_exist then # ist nur sinnvoll, wenn ZB vorliegen
  tmp_t0 := time():
  tmp_l0 := length(thetaD)+length(dD):
  printf("%s: Vereinfache MDH-Geschwindigkeiten. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l0):
  for i from 1 to NJ do
    tmp_t1 := time():
    tmp_l1 := length(thetaD(i,1)) + length(dD(i,1)): # es kann sowieso nur einer der beiden Informationen enthalten
    printf("%s: Vereinfache MDH-Geschw. %d. Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i, tmp_l1):
    thetaD(i,1) := simplify2(thetaD(i,1)):
    dD(i,1)     := simplify2(dD(i,1)):
    tmp_t2 := time():
    tmp_l2 := length(thetaD(i,1)) + length(dD(i,1)):
    printf("%s: MDH-Geschw. %d vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end do:
  tmp_l3 := length(thetaD)+length(dD):
  printf("%s: MDH-Geschwindigkeiten vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l0, tmp_l3, tmp_t2-tmp_t0):
end if:

# Ausdruck für Zeitableitungen der MDH-Winkel exportieren
if codegen_act then
  MatlabExport(convert_t_s(thetaD), sprintf("../codeexport/%s/tmp/velocity_mdh_angles_matlab.m", robot_name), codegen_opt):
  MatlabExport(convert_t_s(dD), sprintf("../codeexport/%s/tmp/velocity_mdh_deltaz_matlab.m", robot_name), codegen_opt):
end if:
# Ausdruck für Maple speichern
save thetaD, dD, sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):




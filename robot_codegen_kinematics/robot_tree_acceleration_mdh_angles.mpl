
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
read "../helper/proc_simplify2":
read "../robot_codegen_definitions/robot_env":
printf("Generiere Beschleunigung für %s\n", robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):  
kin_constraints_exist := kin_constraints_exist: # nur zum Abschätzen der Komplexität
;
# Term-Vereinfachungen einstellen
if not assigned(simplify_options) or simplify_options(5)=-1 then # Standard-Einstellungen:
  if not kin_constraints_exist then # normale serielle Ketten und Baumstrukturen
    use_simplify := 0: # Standardmäßig aus
  else # mit kinematischen Zwangsbedingungen
    use_simplify := 1: # standardmäßig simplify-Befehle anwenden
  end if:
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(5): # fünfter Eintrag ist für Beschleunigung
end if:

# Ergebnisse der Geschwindigkeit laden
read sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):
thetaD:= thetaD:
dD:=dD:
# Lade Marker für Existenz kinematischer Zwangsbedingungen (Verknüpfung von MDH-Gelenkwinkeln durch verallgemeinerte Koordinaten)
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
kin_constraints_exist := kin_constraints_exist:


# Zeitableitung der Drehwinkelgeschwindigkeit berechnen
# Im Falle kinematischer Zwangsbedingungen wurden diese schon im Arbeitsblatt für die Geschwindigkeit eingesetzt
# und müssen hier nicht mehr betrachtet werden.
# Zeitableitung der Geschwindigkeit in Abhängigkeit der verallgemeinerten Koordinaten 
thetaD_qt := thetaD:
dD_qt := dD:
thetaDD := Matrix(NJ, 1):
dDD:= Matrix(NJ, 1):

for i from 1 to NJ do:
  thetaDD(i,1):= diff(thetaD_qt(i,1),t):
  dDD(i,1)    := diff(dD_qt(i,1),t):
end do:
if kin_constraints_exist then # ist nur rechenaufwändig, wenn ZB vorliegen
  printf("%s. Zweite Zeitableitung der MDH-Winkel gebildet.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:
# Terme vereinfachen

if use_simplify>=1 and kin_constraints_exist then # ist nur sinnvoll, wenn ZB vorliegen
  tmp_t0 := time():
  tmp_l0 := length(thetaDD)+length(dDD):
  printf("%s: Vereinfache MDH-Beschleunigungen. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l0):
  for i from 1 to NJ do
    tmp_t1 := time():
    tmp_l1 := length(thetaDD(i,1)) + length(dDD(i,1)): # es kann sowieso nur einer der beiden Informationen enthalten
    printf("%s: Vereinfache MDH-Beschl. %d. Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i, tmp_l1):
    thetaDD(i,1) := simplify2(thetaDD(i,1)):
    dDD(i,1)     := simplify2(dDD(i,1)):
    tmp_t2 := time():
    tmp_l2 := length(thetaDD(i,1)) + length(dDD(i,1)):
    printf("%s: MDH-Beschl. %d vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end do:
  tmp_l3 := length(thetaDD)+length(dDD):
  printf("%s: MDH-Beschleunigungen vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l0, tmp_l3, tmp_t2-tmp_t0):
end if:
# Ausdruck für Zeitableitungen der Beschleunigungen exportieren
if codegen_act then
  MatlabExport(convert_t_s(thetaDD), sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_matlab.m", robot_name), codegen_opt):
  MatlabExport(convert_t_s(dDD), sprintf("../codeexport/%s/tmp/acceleration_mdh_deltaz_matlab.m", robot_name), codegen_opt):
end if:
# Ausdruck für Maple speichern
save thetaDD, dDD, sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_maple.m", robot_name):
save thetaDD, dDD, sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_maple", robot_name):


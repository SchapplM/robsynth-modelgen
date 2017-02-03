# Erstelle Liste aller Kinematikparameter
# Init
# Dieses Arbeitsblatt erstellt einen Vektor mit allen Kinematikparameter
# Es werden nur die Kinematikparameter eingetragen, die ungleich Null sind, also z.B. keine MDH-Parameter die Länge oder Winkel Null sind.
# 
# TODO: Liste mit Kinematikparametern muss immer die selbe Reihenfolge haben. Am besten in der Form a,alpha,qoffset,d,b,beta,kc. Momentan scheint die Reihenfolge zufällig, aber reproduzierbar.
# 
# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100): # Damit der ganze Parametervektor ausgegeben werden kann
with(LinearAlgebra):
read "../helper/proc_MatlabExport":
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
printf("Generiere Kinematik-Parametervektor für %s\n",robot_name):
# Parameter der Zwangsbedingungen lesen
kin_constraints_exist := false:
constrfile := sprintf("../codeexport/%s/kinematic_constraints_symbols_list_maple", robot_name):
if FileTools[Exists](constrfile) then
  read constrfile:
  kin_constraints_exist := true:
end if:
# Liste der Kinematikparameter erstellen
# Alle einzelnen MDH-Parametervektoren stapeln
pkin_tmp1 := <a;alpha;d;qoffset;b;beta>:
# Kinematikparameter für kinematische Zwangsbedingungen hinzufügen (falls vorhanden)
if kin_constraints_exist then
  pkin_tmp1 := <pkin_tmp1;Transpose(kc_symbols)>:
  np_kc := ColumnDimension(kc_symbols):
else
  np_kc := 0:# Anzahl der Kinematikparameter für die Zwangsbedingungen
end if:
# Alle Symbole herausfinden
# TODO: Liste mit besserer Reihenfolge (siehe oben)
nms:=convert(indets(pkin_tmp1,name),list):
# Laufende Nummer für Parameterliste
kk := 0:
# Matrix mit Platz für alle Kinematikparameter (falls alle ungleich Null)
pkin_tmp2 := Matrix(NJ*6+np_kc,1):
# Pi aus Liste entfernen (wird im Code sowieso automatisch eingesetzt)
for i from 1 to ColumnDimension(nms) do
  if nms[i] = Pi then
    next:
  end if:
  # Parameter zur Liste hinzufügen
  kk := kk + 1:
  pkin_tmp2[kk,1] := nms[i]:
end do:
# Ausgabevariable belegen
pkin := Matrix(pkin_tmp2[1..kk,1],kk,1):
printf("Kinematik-Parameter für %s:\n", robot_name):
pkin;
# Ergebnis speichern
# Für Generierung des Kinematikparametervektors in Matlab
MatlabExport(pkin, sprintf("../codeexport/%s/parameter_kin_matlab.m", robot_name), 2):
# Für schnelle Erkennung der Dimension zum Auslesen durch Bash-Skripte
save pkin, sprintf("../codeexport/%s/parameter_kin", robot_name):


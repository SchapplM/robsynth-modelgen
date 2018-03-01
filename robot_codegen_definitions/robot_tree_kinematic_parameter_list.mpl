
# Erstelle Liste aller Kinematikparameter
# Init
# Dieses Arbeitsblatt erstellt einen Vektor mit allen Kinematikparameter
# Es werden nur die Kinematikparameter eingetragen, die ungleich Null sind, also z.B. keine MDH-Parameter die Länge oder Winkel Null sind.
# 
# TODO: Liste mit Kinematikparametern muss immer die selbe Reihenfolge haben. Am besten in der Form a,alpha,qoffset,d,b,beta,kc. Momentan scheint die Reihenfolge zufällig, aber reproduzierbar.
# 
# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100): # Damit der ganze Parametervektor ausgegeben werden kann
;
with(LinearAlgebra):
read "../helper/proc_MatlabExport":
read "../helper/proc_convert_t_s":
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
printf("Generiere Kinematik-Parametervektor für %s\n",robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_twist_definitions", robot_name):
# Parameter der Zwangsbedingungen lesen
kin_constraints_exist := false:
kc_symbols2 := []:
# Für explizite Zwangsbedingungen
constrfile := sprintf("../codeexport/%s/tmp/kinematic_constraints_symbols_list_maple", robot_name):
if FileTools[Exists](constrfile) then
  read constrfile:
  printf("Symbole der expliziten Zwangsbedingungen aus %s gelesen.\n", constrfile):
  kc_symbols2 := kc_symbols:
  kin_constraints_exist := true:
end if:
# Parameter für implizite Zwangsbedingungen (dadurch können auch neue Konstanten hinzugefügt werden)
constrfile := sprintf("../codeexport/%s/tmp/kinematic_implicit_constraints_symbols_list_maple", robot_name):
if FileTools[Exists](constrfile) then
  read constrfile:
  printf("Symbole der impliziten Zwangsbedingungen aus %s gelesen.\n", constrfile):
  kc_symbols2 := <kc_symbols2| kc_symbols>:
  kin_constraints_exist := true:
end if:
# Liste der Kinematikparameter erstellen
# Alle einzelnen MDH-Parametervektoren stapeln
pkin_tmp1 := <a;alpha;d;theta; qoffset;b;beta>:
# Kinematikparameter für kinematische Zwangsbedingungen hinzufügen (falls vorhanden)
if kin_constraints_exist then
  pkin_tmp1 := <pkin_tmp1;Transpose(kc_symbols2)>:
  np_kc := ColumnDimension(kc_symbols2):
else
  np_kc := 0:# Anzahl der Kinematikparameter für die Zwangsbedingungen
end if:
# Alle Symbole herausfinden
# Durch den Befehl fallen die vektoriell angesprochenen Gelenkkoordinaten (qJ) weg.
# TODO: Liste mit besserer Reihenfolge (siehe Dateikopf)
nms:=convert(indets(pkin_tmp1,name),list):
# Laufende Nummer für Parameterliste
kk := 0:
# Matrix mit Platz für alle Kinematikparameter (falls alle ungleich Null)
pkin_tmp2 := Matrix(NJ*6+np_kc,1):
# Terme aus Liste entfernen
for i from 1 to ColumnDimension(nms) do
  if nms[i] = Pi then # (wird im Code sowieso automatisch eingesetzt)
    next:
  end if:
  if nms[i] = t then # sollte eigentlich gar nicht drin sein können
    next:
  end if:
  # Parameter zur Liste hinzufügen
  kk := kk + 1:
  pkin_tmp2[kk,1] := nms[i]:
end do:
# Konstante Winkel, die aus MDH-Definition und nicht aus den ZB kommen umwandeln.
q_s := Matrix(1,1,0): q_t :=q_s:
pkin_tmp1 := convert_t_s(pkin_tmp1):
# Konvertierung nochmals durchführen (damit nicht delta8 und delta8s als verschiedene Symbole auftauchen)
pkin_tmp2 :=convert_t_s(pkin_tmp2):
# Ausgabevariable belegen
pkin := Matrix(pkin_tmp2[1..kk,1],kk,1):
# Symbole nochmals gruppieren
pkin := Transpose(Matrix(convert(indets(pkin,name),list))):
# Ergebnis speichern
# Für Generierung des Kinematikparametervektors in Matlab
MatlabExport(pkin, sprintf("../codeexport/%s/tmp/parameter_kin_matlab.m", robot_name), 2):
# Für schnelle Erkennung der Dimension zum Auslesen durch Bash-Skripte
save pkin, sprintf("../codeexport/%s/tmp/parameter_kin", robot_name):
# Ausgabe
# MDH-Tabelle ausgeben
interface(rtablesize=100):
Test := <<seq(i, i=1..NJ)> | sigma| mu| beta| b | alpha | a | theta | d | v>:
Test:=<<"i" | "sigma" | "mu"|"beta"|"b"|"alpha"|"a"|"theta"|"d"|"v">,Test>:
printf("MDH-Tabelle für %s:\n", robot_name):
Test;
printf("Kinematik-Parameter für %s: %dx%d\n", robot_name, RowDimension(pkin), ColumnDimension(pkin)):
pkin;


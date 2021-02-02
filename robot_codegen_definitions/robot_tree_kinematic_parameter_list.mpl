
# 
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
codegen_act := true:
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_twist_definitions", robot_name):
robot_name_OL := robot_name: # Zusätzliche Variable zur Abgrenzung für implizite Zwangsbedingungen
;
#  Schalte um zwischen impliziten ZB und normalen Systemen
if FileTools[Exists]("../workdir/tbmode") then
  read "../workdir/tbmode": # Datei tbmode wird von Bash-Skripten erstellt und zeigt den aktuellen Modus an
else
  tbmode := "serial":
end if:
if tbmode = "implicit" then
  printf("Modus für IC ist aktiv. Bestimme Kinematikparameter für System mit impliziten ZB.\n"):
  read "../robot_codegen_definitions/robot_env_IC":
end if:
printf("Generiere Kinematik-Parametervektor für %s\n",robot_name):
# Parameter der Zwangsbedingungen lesen
kin_constraints_exist := false:
kc_symbols2 := []:
# Für explizite Zwangsbedingungen
constrfile := sprintf("../codeexport/%s/tmp/kinematic_constraints_symbols_list_maple", robot_name_OL):
if FileTools[Exists](constrfile) then
  read constrfile:
  printf("Symbole der expliziten Zwangsbedingungen aus %s gelesen.\n", constrfile):
  kc_symbols2 := kc_symbols:
  kin_constraints_exist := true:
end if:
# Parameter für implizite Zwangsbedingungen (dadurch können auch neue Konstanten hinzugefügt werden)
constrfile := sprintf("../codeexport/%s/tmp/kinematic_implicit_constraints_symbols_list_maple", robot_name):
# Für implizite Zwangsbedingungen ist "robot_name" das System mit IC und nicht mit OL o.ä.
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
# Zuordnung zwischen Kinematikparametern und MDH-Parametern
# Mit der Zuordnung kann der Vektor der Kinematikparameter (der an jede Funktion übergeben wird) aus den MDH-Parametern bestimmt werden
# TODO: Dieser Ansatz funktioniert nur, wenn keine Additionen von Variablen in den MDH-Parametern enthalten sind.
# Schritt 1: pkin_tmp1 nachverarbeiten: Gelenkvariablen und Pi entfernen (damit der solve-Befehl weniger zu verarbeiten hat)
pkin_tmp3 := pkin_tmp1: # Variable, die gestapelt eine nachbearbeitete Version aller MDH-Parameter enthält
;
for i from 1 to RowDimension(pkin_tmp3) do
  # q entfernen
  for j from 1 to RowDimension(qJ_t) do
    pkin_tmp3(i,1) := subs({qJ_t(j,1)=0}, pkin_tmp3(i,1)):
  end do:
  # Term Null setzen, wenn keine Variablen drin stehen (z.B. nur Pi)
  nms:=convert(indets(pkin_tmp3(i),name),list):
  var_da := 0: # Zähler, ob Variablen enthalten sind
  for k from 1 to ColumnDimension(nms) do
    if nms[k] = Pi then # Pi ist Konstante und funktioniert später nicht im Solve-Befehl
      next:
    end if:
    var_da := 1:
  end do:
  if var_da = 0 then
    pkin_tmp3(i,1) := 0:
  end if:
end do:

pkin_subs_mdh:=copy(pkin): # ohne "copy" wird anscheinend verlinkt und der Rest geht nicht
;
pkt3i := Matrix(RowDimension(pkin_tmp3), 1): # Index-Vektor für pkin_tmp3
;
for i from 1 to RowDimension(pkin_tmp3) do
  pkt3i(i,1):=parse(sprintf("pp%03d", i)): # Hilfsgröße zum späteren Ersetzen
end do:
# Ersetze 

for i from 1 to RowDimension(pkin_tmp3) do
  if pkin_tmp3(i) = 0 then
    next:
  end if:
  for j from 1 to RowDimension(pkin) do
    # printf("i=%d, j=%d\n", i, j):
    erg := solve({pkin_tmp3(i)=pkt3i(i)}, pkin(j)):
    if ArrayTools:-NumElems(erg) = 0 then
      next:
    end if:
    # printf("i=%d\n", i):
    pkin_subs_mdh(j) := subs({lhs(erg[1])=rhs(erg[1])}, pkin_subs_mdh(j)):
    break: # Der erste Fund reicht
  end do:
end do:

# Ergebnis nachverarbeiten
# Platzhalter ("ph") generieren (für die Eingabeargumente der Matlab-Funktion zur Berechnung der Parameter)
beta_ph := Matrix(NJ,1):
b_ph := Matrix(NJ,1):
alpha_ph := Matrix(NJ,1):
a_ph := Matrix(NJ,1):
theta_ph := Matrix(NJ,1):
d_ph := Matrix(NJ,1):
qoffset_ph := Matrix(NJ,1):
for i from 1 to NJ do
  beta_ph(i) := parse(sprintf("beta_mdh(%d)", i)):
  b_ph(i) := parse(sprintf("b_mdh(%d)", i)):
  alpha_ph(i) := parse(sprintf("alpha_mdh(%d)", i)):
  a_ph(i) := parse(sprintf("a_mdh(%d)", i)):
  theta_ph(i) := parse(sprintf("theta_mdh(%d)", i)):
  d_ph(i) := parse(sprintf("d_mdh(%d)", i)):
  qoffset_ph(i) := parse(sprintf("qoffset_mdh(%d)", i)):
end do:
# Platzhalter-Vektor zur Ersetzung der gestapelten MDH-Parameter
pkt1_ph := <a_ph;alpha_ph;d_ph;theta_ph; qoffset_ph;b_ph;beta_ph>:
# Kinematikparameter für kinematische Zwangsbedingungen hinzufügen (falls vorhanden)
#TODO (aktuell wird das Problem mit der NaN-Ersetzung unten umgangen).
;
# Parameter-Indizes mit den Platzhaltern für die Matlab-Funktion ersetzen
for i from 1 to RowDimension(pkin_subs_mdh) do
  for j from 1 to RowDimension(pkt1_ph) do
    pkin_subs_mdh(i) := subs( {pkt3i(j)=pkt1_ph(j)}, pkin_subs_mdh(i) ):
  end do:
end do:
# Ersetze alle noch nicht ersetzten Kinematikparameter mit NaN. Ansonsten ist der Matlab-Code nicht lauffähig und es treten nachgelagerte Probleme auf.
for i from 1 to RowDimension(pkin_subs_mdh) do
  for j from 1 to RowDimension(pkt3i) do
    if has(pkin_subs_mdh(i), pkt3i(j)) then
      printf("Die Variable %s kommt noch in der Umrechnung mdh->pkin vor. Vermutlich Fehler beim substituieren. Setze NaN.\n", String(pkt3i(j))):
      pkin_subs_mdh(i) := subs( {pkt3i(j)=NaN}, pkin_subs_mdh(i) ):
    end if:
  end do:
  for j from 1 to RowDimension(kintmp_s) do
    if has(pkin_subs_mdh(i), kintmp_s(j)) then
      printf("Die Variable %s kommt noch in der Umrechnung mdh->pkin vor. Vermutlich Fehler beim substituieren. Setze NaN.\n", String(kintmp_s(j))):
      pkin_subs_mdh(i) := subs( {kintmp_s(j)=NaN}, pkin_subs_mdh(i) ):
    end if:

  end do:
end do:

# Folgender Term muss immer erzeugt werden (wird für Bash-Skripte benötigt. Sonst dort Fehler und Abbruch).
interface(warnlevel=0): # Unterdrücke die folgende Warnung (weil MDH-Parameter als Funktionsnamen interpretiert werden. Code funktioniert trotzdem.)
MatlabExport(pkin_subs_mdh, sprintf("../codeexport/%s/tmp/parameter_kin_from_mdh_matlab.m", robot_name), 2):
interface(warnlevel=3):

# Ausgabe
# MDH-Tabelle ausgeben
interface(rtablesize=100):
Test := <<seq(i, i=1..NJ)> | sigma| mu| beta| b | alpha | a | theta | d | v>:
Test:=<<"i" | "sigma" | "mu"|"beta"|"b"|"alpha"|"a"|"theta"|"d"|"v">,Test>:
printf("MDH-Tabelle für %s:\n", robot_name):
Test;
printf("Kinematik-Parameter für %s: %dx%d\n", robot_name, RowDimension(pkin), ColumnDimension(pkin)):
Transpose(pkin);


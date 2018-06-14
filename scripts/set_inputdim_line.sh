#!/bin/bash -e
#
# Schreibe eine Hilfszeile für die Code-Generierung in eine Matlab-Funktion.
# Die Hilfszeile enthält Informationen über die Dimension der Eingabeargumente
# (Marker %#cgargs, der von matlabfcn2mex.m benutzt wird.
# Dieses Skript wird auf Funktionen angewendet, die nur den Funktionskopf beinhalten.
#
# Dieses Skript muss gesourced werden und in der Toolbox-Umgebung aufgerufen werden
# 
# TODO: Es funktioniert noch nicht, wenn die Platzhalter (z.B. "%RN%) noch gesetzt sind; werden als Kommentar erkannt.
#
# Eingabe:
# 1: Pfad zur Matlab-Funktion
# 2: Zusätzliche Eingabedaten, die nur bei wenigen Funktionen auftreten und nicht gelistet sind.

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-06
# (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

f=$1
extraargs=$2
# Alle Kommentare entfernen (Beginnen mit "%")
tmp=`grep -o '^[^%]*' $f`

# Mehrzeilige Zeilen zusammenfassen
# Ist bereits mit dem Befehl getan. Drei Punkte entfernen für Zeilenumbruch
tmp=`echo $tmp | sed s/[...]//g`

# Zeile der Übergabeargumente bekommen
# Die Zeile steht zwischen den Klammern nach function
# https://stackoverflow.com/questions/18219030/how-to-extract-the-content-between-two-brackets-by-using-grep
tmp=`echo $tmp | grep -oP '\(\K[^\)]+'`

# Ersetze die Variablennamen mit der Beispielzuweisung (Nullen der richtigen Dimension)
DIMLIST="
qJ|${robot_NQJ},1
qJD|${robot_NQJ},1
qJDD|${robot_NQJ},1
g|3,1
r_base|3,1
phi_base|3,1
V_base|6,1
A_base|6,1
xD_base|6,1
xDD_base|6,1
pkin|${robot_NKP},1
m|${robot_NL},1
rSges|${robot_NL},3
mrSges|${robot_NL},3
Icges|${robot_NL},6
Ifges|${robot_NL},6"

# zusätzliche Ausdrücke anhängen
extraargs=${extraargs//;/$'\n'}
for exp in "$extraargs"; do
  DIMLIST="$DIMLIST
    $exp"
done

# Ausdruck mit Kommas umschließen, damit nächste Ersetzung funktioniert.
tmp=",$tmp,"

# Ursprünglichen Variablennamen mit dem zeros-Ausdruck ersetzen
for dz in $DIMLIST; do
  dz=`echo $dz | sed "s/ //g"`
  vn=$(echo $dz | cut -f1 -d\|)
  d="zeros($(echo $dz | cut -f2 -d\|))"
  #echo "$vn -> $d"
  # Finde den Namen der Übergabevariablen (umschlossen von >=0 Nicht-Alphanumerischen Zeichen) und ersetze ihn durch den cgargs-Platzhalter
  tmp=`echo $tmp | sed "s/\([^a-zA-Z0-9]\)$vn\([^a-zA-Z0-9]\)/\1$d\2/"`
done

# Umschließende Kommas und Leerzeichen wieder entfernen
tmp=`echo $tmp | sed "s/^,//" | sed "s/,$//" | sed "s/ //g"`

# Kommentar in die Zieldatei einsetzen
echo "%#cgargs {$tmp}" >> $f

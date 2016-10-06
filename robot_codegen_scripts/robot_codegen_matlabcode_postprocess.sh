#!/bin/bash
# Nachbearbeitung von generiertem Maple-Code für Matlab.
# Dieses Skript sollte nach dem Maple-Codeexport, aber vor der Matlab-Funktionsgenerierung ausgeführt werden.
# Wenn in Maple mit MatlabExport Eine Matrix exportiert wird mit Optimierungsgrad 1, wird die Matlab-Variable als "unknown" bezeichnet und die Dimension wird nicht initialisiert. 
# Damit ist der Code nicht kompilierbar. Füge die Initialisierung hinzu für die Kompilierbarkeit.
# Argumente:
# $1 Zu untersuchende Datei


# ./robot_codegen_matlabcode_postprocess.sh /home/schappler/IRT_REPOS/maple/codeexport/KAS6m4_kinematic_constraints_subsexp_matlab2.m
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

# Suche nach dem Term unknown.
# Wenn in Maple Vektoren oder Matrizen exportiert werden mit Code-Optimierung Stufe 1 (nicht: tryhard), wird keine Gesamt-Variable dafür gebildet.

matfilepath=$1

if [ "$matfilepath" == "" ] || [ ! -f $matfilepath ]; then
  echo "File $f does not exist!"
  exit 1
fi;

# Prüfe ob Datei schon verarbeitet wurde. Dann steht `unknown=NaN(...)` oben  | head -1
teststring1=`grep "unknown=NaN" $matfilepath ` 
if [ "$teststring1" != "" ]; then
  # Dieses Skript wurde schon einmal angewendet. Abbruch.
  exit 0;
fi;

# Prüfe, ob Matrix-Einträge definiert werden
teststring2=`grep "unknown([0-9]*,[0-9]*)" $matfilepath  | head -1`
if [ "$teststring2" != "" ]; then
  # Größten Wert herausfinden
  # Siehe http://stackoverflow.com/questions/918886/how-do-i-split-a-string-on-a-delimiter-in-bash
  imax=0
  jmax=0
  # Suche Zuweisungen auf Variable "unknown" (2D-Matrix) und entferne Anfang und Ende des Suchausdrucks im Ergebnis mit sed
  zeilen=`grep -o "unknown([0-9]*,[0-9]*)" $matfilepath | sed "s/unknown(//g" | sed "s/)//g"`
  for zeile in $zeilen; do
    # Zeile ist nun im Format "i,j". Speichere i,j in einem Array
    arrIN=(${zeile//,/ })
    # Prüfe, ob die jeweiligen Dimensionen größer sind als die gespeicherten
    if [ ${arrIN[0]} -gt $imax ]; then
      imax=${arrIN[0]}
    fi
    if [ ${arrIN[1]} -gt $jmax ]; then
      jmax=${arrIN[1]}
    fi
  done;
  # Initialisieren die Variable "unknown"
  # Siehe http://superuser.com/questions/246837/how-do-i-add-text-to-the-beginning-of-a-file-in-bash/246841
  sed -i "1s/^/unknown=NaN($imax,$jmax)\n/" $matfilepath
fi;

# Ersetze exportierte Inert-Funktionen aus Maple
# Bei inert-Funktionen ist dem Funktiosnamen ein "%" vorgesetzt.
# Diese Funktionen werten einen Ausdruck nicht sofort aus und ermöglichen schnellere Berechnungen.
teststring=`grep "%arctan" $matfilepath | head -1`
if [ "$teststring" != "" ]; then
  sed -i "s/%arctan/atan2/g" $matfilepath
fi;

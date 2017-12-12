#!/bin/bash -e
# Nachbearbeitung von generiertem Maple-Code für Matlab.
# Dieses Skript sollte nach dem Maple-Codeexport, aber vor der Matlab-Funktionsgenerierung ausgeführt werden.
# Wenn in Maple mit MatlabExport Eine Matrix exportiert wird mit Optimierungsgrad 1, wird die Matlab-Variable als "unknown" bezeichnet und die Dimension wird nicht initialisiert. 
# Damit ist der Code nicht kompilierbar. Füge die Initialisierung hinzu für die Kompilierbarkeit.
# Argumente:
# $1 Zu untersuchende Datei

# TODO: Prüfung auf unknown beeinflusst nicht die Prüfung auf %arctan

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

# Suche nach dem Term unknown.
# Wenn in Maple Vektoren oder Matrizen exportiert werden mit Code-Optimierung Stufe 1 (nicht: tryhard), wird keine Gesamt-Variable dafür gebildet.

matfilepath=$1

if [ "$matfilepath" == "" ] || [ ! -f $matfilepath ]; then
  echo "File $f does not exist!"
  exit 1
fi;

# Prüfe ob Datei schon verarbeitet wurde. Dann steht `unknown=NaN(...)` oben  | head -1
teststring1=`grep "unknown=NaN" $matfilepath ` || true
if [ "$teststring1" != "" ]; then
  # Dieses Skript wurde schon einmal angewendet. Abbruch.
  exit 0;
fi;

# Prüfe, ob Matrix-Einträge definiert werden
#cat $matfilepath  | head -1
#head -1 $matfilepath
teststring2=`grep "unknown([0-9]*,[0-9]*)" $matfilepath  | head -1` # Für 2D-Format
teststring3=`grep "unknown([0-9]*)" $matfilepath  | head -1` # Für 1D-Format
if [ "$teststring2" != "" ] || [ "$teststring3" != "" ]; then
  # Größten Wert herausfinden
  # Siehe http://stackoverflow.com/questions/918886/how-do-i-split-a-string-on-a-delimiter-in-bash
  imax=0
  jmax=0
  # Suche Zuweisungen auf Variable "unknown" (2D-Matrix) und entferne Anfang und Ende des Suchausdrucks im Ergebnis mit sed
  if [ "$teststring2" != "" ]; then # Matrix-Format (i,j)
    zeilen=`grep -o "unknown([0-9]*,[0-9]*)" $matfilepath | sed "s/unknown(//g" | sed "s/)//g"`
  else # Vektor-Format (i)
    zeilen=`grep -o "unknown([0-9]*)" $matfilepath | sed "s/unknown(//g" | sed "s/)//g"`
  fi
  for zeile in $zeilen; do
    zeile="$zeile,1" # Zusatz-Dimension anhängen, damit egal ist, ob es 2D- oder 1D-Eingabe ist
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
  # Initialisieren die Variable "unknown" am Anfang
  # Siehe http://superuser.com/questions/246837/how-do-i-add-text-to-the-beginning-of-a-file-in-bash/246841
  sed -i "1s/^/unknown=NaN($imax,$jmax);\n/" $matfilepath
  # Nochmaliges Setzen der Variable "unknown" am Ende (damit Variablenname von Skript robot_codegen_matlabfcn_postprocess.sh erkannt wird)
  echo "unknown = unknown; %#ok<ASGSL> für automatische Verarbeitung" >> $matfilepath
fi;

# Ersetze exportierte Inert-Funktionen aus Maple
# Bei inert-Funktionen ist dem Funktiosnamen ein "%" vorgesetzt.
# Diese Funktionen werten einen Ausdruck nicht sofort aus und ermöglichen schnellere Berechnungen.
teststring=`grep "%arctan" $matfilepath | head -1` || true
if [ "$teststring" != "" ]; then
  sed -i "s/%arctan/atan2/g" $matfilepath
fi;

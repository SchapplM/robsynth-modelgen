#!/bin/bash 
# Nachbearbeitung einer automatisch erstellten Matlab-Funktion
#
# Argumente:
# mfcndat: Pfad der zu bearbeitenden Datei
# replacelastassignment: Falls 0 wird die letzte Variablenzuweisung in der Datei nicht geändert (für Skripte)
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover


repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
source $repo_pfad/robot_codegen_definitions/robot_env.sh

mfcndat=$1 # Dateipfad als Übergabeargument
replacelastassignment=$2

# Ersetze Platzhalterausdrücke $RN$, $NJ$, $NL$
# Hier müssen normale und nicht einfache Anführungszeichen für `sed` genommen werden. Sonst wird das $-Zeichen für die Variable als Text interpretiert...
sed -i "s/%RN%/$robot_name/g" $mfcndat
sed -i "s/%NJ%/$robot_NJ/g" $mfcndat
sed -i "s/%NL%/$robot_NL/g" $mfcndat
sed -i "s/%NMPV%/$robot_NMPV/g" $mfcndat

if [ "$replacelastassignment" != "0" ]; then # vergleiche strings, da das Argument auch leer sein könnte
  # Ersetze Variablennamen des letzten Ergebnisses des generierten Codes
  # prüfe, welches die Ausgabevariable der Funktion ist
  varname_fcn=`grep "=" $mfcndat | head -1 | sed 's/function \(.*\)=.*/\1/'`
  # prüfe, welches die Ausgabevariable des Maple-exportierten Codes ist
  varname_tmp=`grep "=" $mfcndat | tail -1 | sed 's/\(.*\)=.*/\1/'`
  # Ergänze die Zuweisung der Ausgabevariablen
  echo "$varname_fcn = $varname_tmp;" >> $mfcndat
fi

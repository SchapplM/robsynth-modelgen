#!/bin/bash -e
# Nachbearbeitung einer automatisch erstellten Matlab-Funktion
#
# Argumente:
# 1: mfcndat: Pfad der zu bearbeitenden Datei
# 2: replacelastassignment: Falls 0 wird die letzte Variablenzuweisung in der Datei nicht geändert (für Skripte). Standard: 1
# 3: lastassignmentvector: Falls 1 wird die letzte Variablenzuweisung als Vektor (stehend) erzwungen. Der autogenerierte Maple-Code ist bei Vektoren manchmal liegend, manchmal stehend. Standard: 0
# 4: subsvardat: Matlab-Skript mit Ersetzungsausdrücken für die Maple-Variablen. Kann die ursprünglich vorgesehene Variableninitialisierung ersetzen. Standard: Leer
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover


repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
tmp_head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
source $repo_pfad/robot_codegen_definitions/robot_env.sh

mfcndat=$1 # Dateipfad als Übergabeargument
replacelastassignment=$2
lastassignmentvector=$3
subsvardat=$4

# Standardwerte für Eingabe
if [ "$replacelastassignment" == "" ]; then
  replacelastassignment=1
fi
if [ "$lastassignmentvector" == "" ]; then
  lastassignmentvector=0
fi
if [ "$subsvardat" == "" ]; then
  subsvardat=""
fi

# Namen der Funktion generieren
fntmp=$(basename "$mfcndat")
FN="${fntmp%.*}"

# Setze Beschreibung der Eingabegrößen ein (diese sind für viele Funktionen identisch)
sed -i "/% %INPUT_M%/r $tmp_head_pfad/robot_matlabtmp_comment_input_m.m" $mfcndat
sed -i "/%INPUT_M%/d" $mfcndat
sed -i "/% %INPUT_R%/r $tmp_head_pfad/robot_matlabtmp_comment_input_r.m" $mfcndat
sed -i "/%INPUT_R%/d" $mfcndat
sed -i "/% %INPUT_MR%/r $tmp_head_pfad/robot_matlabtmp_comment_input_mr.m" $mfcndat
sed -i "/%INPUT_MR%/d" $mfcndat
sed -i "/% %INPUT_IC%/r $tmp_head_pfad/robot_matlabtmp_comment_input_Ic.m" $mfcndat
sed -i "/%INPUT_IC%/d" $mfcndat
sed -i "/% %INPUT_IF%/r $tmp_head_pfad/robot_matlabtmp_comment_input_If.m" $mfcndat
sed -i "/%INPUT_IF%/d" $mfcndat
sed -i "/% %INPUT_MDPFIXB%/r $tmp_head_pfad/robot_matlabtmp_comment_input_MDPFIXB.m" $mfcndat
sed -i "/%INPUT_MDPFIXB%/d" $mfcndat
sed -i "/% %INPUT_PKIN%/r $tmp_head_pfad/robot_matlabtmp_comment_input_pkin.m" $mfcndat
sed -i "/%INPUT_PKIN%/d" $mfcndat
sed -i "/% %INPUT_QJ%/r $tmp_head_pfad/robot_matlabtmp_comment_input_qJ.m" $mfcndat
sed -i "/%INPUT_QJ%/d" $mfcndat
sed -i "/% %INPUT_QJD%/r $tmp_head_pfad/robot_matlabtmp_comment_input_qJD.m" $mfcndat
sed -i "/%INPUT_QJD%/d" $mfcndat
sed -i "/% %INPUT_QJDD%/r $tmp_head_pfad/robot_matlabtmp_comment_input_qJDD.m" $mfcndat
sed -i "/%INPUT_QJDD%/d" $mfcndat
sed -i "/% %INPUT_RB%/r $tmp_head_pfad/robot_matlabtmp_comment_input_rB.m" $mfcndat
sed -i "/%INPUT_RB%/d" $mfcndat
sed -i "/% %INPUT_PHIB%/r $tmp_head_pfad/robot_matlabtmp_comment_input_phiB.m" $mfcndat
sed -i "/%INPUT_PHIB%/d" $mfcndat
sed -i "/% %INPUT_PHIBD%/r $tmp_head_pfad/robot_matlabtmp_comment_input_phiBD.m" $mfcndat
sed -i "/%INPUT_PHIBD%/d" $mfcndat
sed -i "/% %INPUT_XDB%/r $tmp_head_pfad/robot_matlabtmp_comment_input_xDB.m" $mfcndat
sed -i "/%INPUT_XDB%/d" $mfcndat
sed -i "/% %INPUT_XDDB%/r $tmp_head_pfad/robot_matlabtmp_comment_input_xDDB.m" $mfcndat
sed -i "/%INPUT_XDDB%/d" $mfcndat

# Ersetze Platzhalterausdrücke $RN$, $NJ$, $NL$
# Hier müssen normale und nicht einfache Anführungszeichen für `sed` genommen werden. Sonst wird das $-Zeichen für die Variable als Text interpretiert...
sed -i "s/%RN%/$robot_name/g" $mfcndat
sed -i "s/%NQJ%/$robot_NQJ/g" $mfcndat
sed -i "s/%NJ%/$robot_NJ/g" $mfcndat
sed -i "s/%NL%/$robot_NL/g" $mfcndat
sed -i "s/%NMPVFIXB%/$robot_NMPVFIXB/g" $mfcndat
sed -i "s/%NMPVFLOATB%/$robot_NMPVFLOATB/g" $mfcndat
sed -i "s/%FN%/$FN/g" $mfcndat
sed -i "s/%NKP%/$robot_NKP/g" $mfcndat
sed -i "s/%NKCP%/$robot_NKCP/g" $mfcndat
kpstring="pkin=[$(echo "$robot_KP" | sed "s/ /,/g")]';"
sed -i "s/%KPDEF%/$kpstring/g" $mfcndat

if [ "$replacelastassignment" != "0" ]; then # vergleiche strings, da das Argument auch leer sein könnte
  # Ersetze Variablennamen des letzten Ergebnisses des generierten Codes
  # prüfe, welches die Ausgabevariable der Funktion ist (steht oben im Funktionskopf)
  varname_fcn=`grep "function .*=" $mfcndat | sed 's/function \(.*\)=.*/\1/'`
  # prüfe, welches die Ausgabevariable des Maple-exportierten Codes ist (letzte Zuweisung im Code, ganz unten).
  # Die Variable kann entweder direkt, oder indiziert vor dem Gleichheitszeichen stehen.
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $mfcndat`
  # Ergänze die Zuweisung der Ausgabevariablen
  if [ "$lastassignmentvector" == "1" ]; then
    echo "$varname_fcn = $varname_tmp(:);" >> $mfcndat
  else
    echo "$varname_fcn = $varname_tmp;" >> $mfcndat
  fi
fi

# Ersetze einzelne Maple-Variablen im Code
# In der subsvar-Datei stehen Ersetzungsausdrücke zur Umsetzung von Maple-Variablen auf die Matlab-Eingabevariablen der Funktion
if [ "$subsvardat" != "" ] && [ -f "$subsvardat" ]; then
  OLDIFS=$IFS
  IFS=$'\n'
  # Ersetzungsausdrücke sind normale Matlab-Befehle (mit Gleichheitszeichen)
  sedregexp='/./ s:\(.*\) = \(.*\);:\1|\2:g'
  for l in `sed "$sedregexp" "$subsvardat"`; do
    OLDEXP=$(echo $l | cut -f1 -d\|)
    NEWEXP=$(echo $l | cut -f2 -d\|)
    # Ersetze den Ausdruck nur im Matlab-Code und nur für Variablen (nicht überall im Text)
    # (die Variablen werden im Quelltext immer von Zeichen begrenzt, die kein Teil des Variablennamens sein können.)
    # Keine Ersetzung in Kommentarzeilen
    sed -i "/^%/! s/\([^a-zA-Z0-9_]\)$OLDEXP\([^a-zA-Z0-9_]\)/\1$NEWEXP\2/g" $mfcndat
  done
  IFS=$OLDIFS
fi

# Versionsinformationen einfügen an vorgesehene Stelle
versionfile=$tmp_pfad/version_info.head.m
sed -i "/% %VERSIONINFO%/r $versionfile" $mfcndat
sed -i "/%VERSIONINFO%/d" $mfcndat

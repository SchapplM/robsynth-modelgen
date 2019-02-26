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

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover


repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
tmp_head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head

source $repo_pfad/robot_codegen_definitions/robot_env_par.sh
source $repo_pfad/codeexport/${robot_leg_name}/tmp/robot_env.sh
# Erneutes Einlesen des Roboternamens der PKM
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh


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
sed -i "/% %INPUT_M_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_m_parallel.m" $mfcndat
sed -i "/%INPUT_M_P%/d" $mfcndat
sed -i "/% %INPUT_R_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_r_parallel.m" $mfcndat
sed -i "/%INPUT_R_P%/d" $mfcndat
sed -i "/% %INPUT_MR_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_mr_parallel.m" $mfcndat
sed -i "/%INPUT_MR_P%/d" $mfcndat
sed -i "/% %INPUT_IC_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_Ic_parallel.m" $mfcndat
sed -i "/%INPUT_IC_P%/d" $mfcndat
sed -i "/% %INPUT_IF_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_If_parallel.m" $mfcndat
sed -i "/%INPUT_IF_P%/d" $mfcndat
sed -i "/% %INPUT_PKIN%/r $tmp_head_pfad/robot_matlabtmp_comment_input_pkin.m" $mfcndat
sed -i "/%INPUT_PKIN%/d" $mfcndat
sed -i "/% %INPUT_PKIN_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_pkin_parallel.m" $mfcndat
sed -i "/%INPUT_PKIN_P%/d" $mfcndat
sed -i "/% %INPUT_QJ_P%/r $tmp_head_pfad/robot_matlabtmp_comment_input_qJ_parallel.m" $mfcndat
sed -i "/%INPUT_QJ_P%/d" $mfcndat
sed -i "/% %INPUT_XP%/r $tmp_head_pfad/robot_matlabtmp_comment_input_xP.m" $mfcndat
sed -i "/%INPUT_XP%/d" $mfcndat
sed -i "/% %INPUT_XDP%/r $tmp_head_pfad/robot_matlabtmp_comment_input_xDP.m" $mfcndat
sed -i "/%INPUT_XDP%/d" $mfcndat
sed -i "/% %INPUT_XDDP%/r $tmp_head_pfad/robot_matlabtmp_comment_input_xDDP.m" $mfcndat
sed -i "/%INPUT_XDDP%/d" $mfcndat
sed -i "/% %INPUT_LEGFRAME%/r $tmp_head_pfad/robot_matlabtmp_comment_input_legFrame_parallel.m" $mfcndat
sed -i "/%INPUT_LEGFRAME%/d" $mfcndat
sed -i "/% %INPUT_KOPPEL%/r $tmp_head_pfad/robot_matlabtmp_comment_input_koppelP_parallel.m" $mfcndat
sed -i "/%INPUT_KOPPEL%/d" $mfcndat
sed -i "/% %INPUT_RSP%/r $tmp_head_pfad/robot_matlabtmp_comment_input_rSP_parallel.m" $mfcndat
sed -i "/%INPUT_RSP%/d" $mfcndat

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
sed -i "s/%NQJ_P%/$parallel_NQJ_leg/g" $mfcndat
sed -i "s/%N_LEGS%/$parallel_NLEGS/g" $mfcndat
sed -i "s/%N_XP%/$parallel_NX/g" $mfcndat
nges=$((parallel_NQJ_leg + 1))
sed -i "s/%NGES%/$nges/g" $mfcndat
sed -i "s/%NMPVPARA%/$robot_NMPVPARA/g" $mfcndat

# TODO: Dieser Teil ist doppelt zwischen seriell und parallel. Eventuell nur an einer Stelle halten.
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
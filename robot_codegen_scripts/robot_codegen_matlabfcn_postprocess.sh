#!/bin/bash -e 
# Nachbearbeitung einer automatisch erstellten Matlab-Funktion
#
# Argumente:
# mfcndat: Pfad der zu bearbeitenden Datei
# replacelastassignment: Falls 0 wird die letzte Variablenzuweisung in der Datei nicht geändert (für Skripte)
# lastassignmentvector: Falls 1 wird die letzte Variablenzuweisung als Vektor (stehend) erzwungen. Der autogenerierte Maple-Code ist bei Vektoren manchmal liegend, manchmal stehend.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover


repo_pfad=$(pwd)/..
source $repo_pfad/robot_codegen_definitions/robot_env.sh

mfcndat=$1 # Dateipfad als Übergabeargument
replacelastassignment=$2
lastassignmentvector=$3

# Namen der Funktion generieren
fntmp=$(basename "$mfcndat")
FN="${fntmp%.*}"

# Setze Beschreibung der Eingabegrößen ein (diese sind für viele Funktionen identisch)
sed -i "/% %INPUT_M%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_m.m" $mfcndat
sed -i "/%INPUT_M%/d" $mfcndat
sed -i "/% %INPUT_R%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_r.m" $mfcndat
sed -i "/%INPUT_R%/d" $mfcndat
sed -i "/% %INPUT_MR%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_mr.m" $mfcndat
sed -i "/%INPUT_MR%/d" $mfcndat
sed -i "/% %INPUT_IC%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_Ic.m" $mfcndat
sed -i "/%INPUT_IC%/d" $mfcndat
sed -i "/% %INPUT_IF%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_If.m" $mfcndat
sed -i "/%INPUT_IF%/d" $mfcndat
sed -i "/% %INPUT_PKIN%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_pkin.m" $mfcndat
sed -i "/%INPUT_PKIN%/d" $mfcndat
sed -i "/% %INPUT_QJ%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_qJ.m" $mfcndat
sed -i "/%INPUT_QJ%/d" $mfcndat
sed -i "/% %INPUT_QJD%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_qJD.m" $mfcndat
sed -i "/%INPUT_QJD%/d" $mfcndat
sed -i "/% %INPUT_QJDD%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_qJDD.m" $mfcndat
sed -i "/%INPUT_QJDD%/d" $mfcndat
sed -i "/% %INPUT_RB%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_rB.m" $mfcndat
sed -i "/%INPUT_RB%/d" $mfcndat
sed -i "/% %INPUT_PHIB%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_phiB.m" $mfcndat
sed -i "/%INPUT_PHIB%/d" $mfcndat
sed -i "/% %INPUT_XDB%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_xDB.m" $mfcndat
sed -i "/%INPUT_XDB%/d" $mfcndat
sed -i "/% %INPUT_XDDB%/r $repo_pfad/robot_codegen_scripts/tmp_head/robot_matlabtmp_comment_input_xDDB.m" $mfcndat
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

if [ "$replacelastassignment" != "0" ]; then # vergleiche strings, da das Argument auch leer sein könnte
  # Ersetze Variablennamen des letzten Ergebnisses des generierten Codes
  # prüfe, welches die Ausgabevariable der Funktion ist (steht oben im Funktionskopf)
  varname_fcn=`grep "function .*=" $mfcndat | sed 's/function \(.*\)=.*/\1/'`
  # prüfe, welches die Ausgabevariable des Maple-exportierten Codes ist (letzte Zuweisung im Code, ganz unten)
  varname_tmp=`grep "=" $mfcndat | tail -1 | sed 's/\(.*\)=.*/\1/'`
  # Ergänze die Zuweisung der Ausgabevariablen
  if [ "$lastassignmentvector" == "1" ]; then
    echo "$varname_fcn = $varname_tmp(:);" >> $mfcndat
  else
    echo "$varname_fcn = $varname_tmp;" >> $mfcndat
  fi
fi

# Versionsinformationen einfügen an vorgesehene Stelle
# TODO: Versionsdatei nicht jedes Mal neu erzeugen (zu viele Schreibzugriffe)
versionfile=$repo_pfad/robot_codegen_scripts/tmp/version_info.head.m
echo "% Quelle: IRT-Maple-Repo" > $versionfile
now="$(date +'%Y-%m-%d %H:%M')"
printf "%% Datum: $now\n" >> $versionfile
rev=`git rev-parse HEAD`
printf "%% Revision: $rev\n" >> $versionfile
echo "% (C) Institut für Regelungstechnik, Universität Hannover" >> $versionfile

sed -i "/% %VERSIONINFO%/r $versionfile" $mfcndat
sed -i "/%VERSIONINFO%/d" $mfcndat

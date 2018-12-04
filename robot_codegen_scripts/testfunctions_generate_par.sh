#!/bin/bash -e
# Passe die Vorlagen für Dynamik-Testfunktionen an den Roboter an.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Testskripte für Matlabfunktionen"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp

# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

testfcn_pfad=$repo_pfad/codeexport/$robot_name/testfcn

for f in $(find $repo_pfad/robot_codegen_testfunctions_par -name "*.template")
do
  tmpdat_full=$f
  filename="${tmpdat_full##*/}"                      # Strip longest match of */ from start
  # Verzeichnis der Vorlagen-Datei
  dir1="${tmpdat_full:0:${#tmpdat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Neuen Dateinamen generieren
  tmp="${filename/robot/$robot_name}" # Ersetze Begriff robot mit Roboternamen
  filename_new="${tmp/.template/}" # Endung .template entfernen
  
  # Neues Verzeichnis generieren: Jetzt im Ordner codeexport/%RN%/testfcn
  dir2=$testfcn_pfad

  # Datei kopieren
  mkdir -p "$dir2"
  cp $dir1/$filename $dir2/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess_par.sh $dir2/$filename_new 0
done

# Parameter-Generierungsskript anpassen
zieldat=$testfcn_pfad/${robot_name}_varpar_testfunctions_parameter_par.m

# Winkelgrenzen in Parameter-Skript berücksichtigen
qlim_dat=$repo_pfad/robot_codegen_constraints/${robot_name}_kinematic_constraints_matlab.m
if [ -f "$qlim_dat" ]; then
  # TODO: Zeilenumbrüche nach Einsetzen wiederherstellen, Einrückung schön machen.
  REPSTR=`tr "\n" " " < $qlim_dat`
else
  # nur zufällige Winkel
  REPSTR="xP_min = -pi*ones(N_XP,1);xP_max = pi*ones(N_XP,1);q_min = -pi*ones(NQJ,1);q_max = pi*ones(NQJ,1);"
fi;
# Ersetzungsausdruck für Winkelausdruck in Vorlage ersetzen.
sed -i "s|%REPLACE_QDEF%|$REPSTR|" $zieldat

# Zufällige Werte für Kinematikparameter
printf "\n%%%% Zufällige Werte für Kinematikparameter\n" >> $zieldat
for Kp in $robot_KP; do
  echo "$Kp = rand(1,1);" >> $zieldat
done

KP_dat2=$repo_pfad/codeexport/${robot_leg_name}/tmp/parameter_kin_matlab.m
if [ -f "$KP_dat2" ]; then
  printf "\n%% Aus ${robot_leg_name}/parameter_kin_matlab.m\n" >> $zieldat
  cat $KP_dat2 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "pkin = $varname_tmp;" >> $zieldat
  echo "if isempty(pkin)" >> $zieldat
  echo "  pkin = zeros(0,1);" >> $zieldat
  echo "end" >> $zieldat
else
  echo "Kinematik-Parametervektor in $KP_dat2 nicht gefunden"
  exit 1
fi;

# Schreibe Ausgangsvariable (lese Teil aus Vorlage)
quelldat=$repo_pfad/robot_codegen_testfunctions_par/robot_varpar_testfunctions_parameter_par.m.template2
cat $quelldat >> $zieldat

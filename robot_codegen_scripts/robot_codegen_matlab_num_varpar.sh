#!/bin/bash -e
# Passe die Vorlagen für numerische Funktionen an den Roboter an.
# Diese Funktionen sind durch assert-Befehle und hart kodierte
# Zahlen an den Roboter angepasst und dadurch kompilierbar.
# Die allgemeinen Dynamikfunktionen aus der Matlab-Toolbox
# sind nicht kompilierbar und damit etwas langsamer.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Dynamikfunktion (numerisch)"

repo_pfad=$(pwd)/..
template_pfad=$repo_pfad/robot_codegen_scripts/templates_num

# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

fcn_pfad=$repo_pfad/codeexport/$robot_name/matlabfcn

for f in $(find $template_pfad -name "*.template")
do
  tmpdat_full=$f
  filename="${tmpdat_full##*/}"                      # Strip longest match of */ from start
  dir="${tmpdat_full:0:${#tmpdat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Neuen Dateinamen generieren
  tmp="${filename/robot/$robot_name}"
  filename_new="${tmp/.template/}"
  
  # Datei kopieren
  cp $dir/$filename $fcn_pfad/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess.sh $fcn_pfad/$filename_new 0
done


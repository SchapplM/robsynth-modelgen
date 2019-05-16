#!/bin/bash -e
# Starte alle Maple-Skripte nacheinander in der richtigen Reihenfolge für hybriden Roboter mit impliziten Zwangsbedingungen
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad

# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash_IC.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Liste mit Maple-Skripten in der richtigen Reihenfolge für parallelen Roboter

# Kinematische Zwangsbedingungen in impliziter Form
# Werden nach der Kinematik gerechnet. Können also auch hier am Ende kommen
# Die Ergebnisse werden in der Dynamik nicht weiter benutzt (im Gegensatz zu explizit definierten Zwangsbedingungen, die direkt zur Ersetzung dienen).
dateiliste_kindyn="
  ${robot_name}_kinematic_constraints_implicit.mpl
  robot_kinematic_constraints_calculations_implicit.mpl
  "

# Alle Maple-Dateien der Reihe nach ausführen
for mpldat in $dateiliste_kindyn
do
  mpldat_full=$repo_pfad/workdir/$mpldat
  filename="${mpldat_full##*/}"                      # Strip longest match of */ from start
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Maple im Kommandozeilenmodus starten (vorher ins richtige Verzeichnis wechseln)
  echo "Starte Maple-Skript $filename"
  $repo_pfad/scripts/run_maple_script.sh $mpldat_full
done

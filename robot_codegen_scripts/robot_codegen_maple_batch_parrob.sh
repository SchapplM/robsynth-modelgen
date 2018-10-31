#!/bin/bash -e
# Starte alle Maple-Skripte nacheinander in der richtigen Reihenfolge für parallelen Roboter
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad

# Standard-Einstellungen
CG_MINIMAL=0
CG_FIXBONLY=0
CG_FLOATBONLY=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    --minimal)
    CG_MINIMAL=1
    ;;
    --fixb_only)
    CG_FIXBONLY=1
    ;;
    --floatb_only)
    CG_FLOATBONLY=1
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Liste mit Maple-Skripten in der richtigen Reihenfolge für parallelen Roboter
dateiliste_kindyn="
  robot_para_definitions.mpl
  robot_para_rotmat_kinematics.mpl
  "
if [ "$CG_MINIMAL" == "0" ]; then
  dateiliste_kindyn="$dateiliste_kindyn
  robot_para_plattform_rotmat_dynamics_par1.mpl
  robot_para_rotmat_projection_dynamics_par1.mpl
  "
fi;

dateiliste_kindyn="$dateiliste_kindyn
  robot_para_plattform_rotmat_dynamics_par2.mpl
  robot_para_rotmat_projection_dynamics_par2.mpl
  "

  #robot_para_plattform_rotmat_dynamics_regressor.mpl
  #robot_para_rotmat_projection_dynamics_regressor.mpl
  

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

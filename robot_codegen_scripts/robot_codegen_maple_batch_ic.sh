#!/bin/bash -e
# Starte alle Maple-Skripte nacheinander in der richtigen Reihenfolge für hybriden Roboter mit impliziten Zwangsbedingungen
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad

# Standard-Einstellungen
CG_MINIMAL=0
CG_FIXBONLY=0
CG_FLOATBONLY=0
CG_KINEMATICSONLY=0

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
    --kinematics_only)
    CG_KINEMATICSONLY=1
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash_ic.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Aktuellen Modus speichern (für Arbeitsblätter, die im IC- oder Normal-Modus benutzt werden)
echo "tbmode := \"implicit\":" > $repo_pfad/workdir/tbmode

# Liste mit Maple-Skripten in der richtigen Reihenfolge für hybride Roboter

# Kinematische Zwangsbedingungen in impliziter Form
# Werden nach der Kinematik gerechnet. Können also auch hier am Ende kommen
# Die Ergebnisse werden in der Dynamik nicht weiter benutzt (im Gegensatz zu explizit definierten Zwangsbedingungen, die direkt zur Ersetzung dienen).
dateiliste_kindyn="
  ${robot_name}_kinematic_constraints_implicit.mpl
  robot_kinematic_constraints_calculations_implicit.mpl
  "
# Liste der Kinematikparameter. Kann sich bei manuell eingefügten Zwangsbedingungen
# von den Parametern der offenen Baumstruktur unterscheiden.
# Tritt normalerweise nicht bei geschlossenen kinematischen Ketten auf.
dateiliste_kindyn="$dateiliste_kindyn
  robot_tree_kinematic_parameter_list.mpl
"

if [ "$CG_MINIMAL" == "0" ] && [ "$CG_KINEMATICSONLY" == "0" ]; then
  dateiliste_kindyn="$dateiliste_kindyn
      robot_implicit_contraints_rotmat_dynamics_worldframe_par1_invdyn.mpl
      robot_implicit_contraints_rotmat_dynamics_worldframe_par2_invdyn.mpl
      robot_implicit_contraints_rotmat_dynamics_worldframe_par1_grav_inertia.mpl
      robot_implicit_contraints_rotmat_dynamics_worldframe_reg2.mpl
  "
fi;

if [ "$CG_KINEMATICSONLY" == "0" ]; then
  dateiliste_kindyn="$dateiliste_kindyn
      robot_implicit_contraints_rotmat_dynamics_worldframe_par2_grav_inertia.mpl
      robot_implicit_contraints_rotmat_dynamics_worldframe_snew_par2.mpl
  "
fi;

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

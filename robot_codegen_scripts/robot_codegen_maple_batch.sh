#!/bin/bash -e
# Starte alle Maple-Skripte nacheinander in der richtigen Reihenfolge
#
# Argumente:
# --fixb_only
#   Nur Berechnung der Fixed-Base Funktionen.
# --floatb_only
#   Nur Berechnung der Fixed-Base Funktionen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad

# Standard-Einstellungen
CG_FIXBONLY=0
CG_FLOATBONLY=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
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

if [ "$CG_FIXBONLY" == "1" ] && [ "$CG_FLOATBONLY" == "1" ]; then
  echo "Nicht beide Optionen gleichzeitig möglich: fixb_only, floatb_only"
  exit 1
fi;

# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Liste mit Maple-Skripten in der richtigen Reihenfolge
# Skripte für Fixed-Base-Modellierung
dateiliste_kindyn="
    robot_codegen_definitions/robot_tree_floatb_twist_definitions.mpl
"
if [ -f  $repo_pfad/robot_codegen_constraints/${robot_name}_kinematic_constraints.mpl ]; then
	dateiliste_kindyn="$dateiliste_kindyn
		robot_codegen_constraints/${robot_name}_kinematic_constraints.mpl
	"
fi;
# Liste der Kinematikparameter
dateiliste_kindyn="$dateiliste_kindyn
	robot_codegen_definitions/robot_tree_kinematic_parameter_list.mpl
"

# Fixed-Base Terme
if ! [ "$CG_FLOATBONLY" == "1" ]; then
  # Kinematik, Geschwindigkeiten
  dateiliste_kindyn="$dateiliste_kindyn
      robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mpl
      robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
      robot_codegen_kinematics/robot_tree_velocity_mdh_angles.mpl
      robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
      robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_linkframe.mpl
  "

  # Jacobi-Matrizen
  for (( ib=1; ib<=$robot_NL; ib++ ))
  do
    dateiliste_kindyn="$dateiliste_kindyn
          robot_codegen_kinematics/robot_tree_rotmat_jacobian_baseframe_body${ib}.mpl
    "
  done

  # Dynamik
  dateiliste_kindyn="$dateiliste_kindyn
      robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
      robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
      robot_codegen_energy/robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
      robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1.mpl
      robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl
  "


  # Skripte für Regressorform
  # TODO: Anpassung hier, wenn Regressorformen auch für Floating Base verfügbar
  dateiliste_kindyn="$dateiliste_kindyn
      robot_codegen_energy/robot_chain_fixb_rotmat_energy_regressor.mpl
      robot_codegen_dynamics/robot_chain_floatb_rotmat_dynamics_regressor_pv2.mpl
      robot_codegen_dynamics/robot_chain_floatb_rotmat_dynamics_regressor_minpar.mpl
  "

  # Initialisiere zusätzliche Maple-Skripte speziell für dieses System (benutzerdefiniert)
  # Mit Basis-Methode "twist"
  addlistfile=$repo_pfad/robot_codegen_additional/scripts/${robot_name}_maple_additional_worksheet_list_twist
  if [ -f $addlistfile ]; then
    dateiliste_kindyn="$dateiliste_kindyn `cat $addlistfile`"
  fi;

fi; # fixb


if ! [ "$CG_FIXBONLY" == "1" ]; then
  # Skripte für Floating-Base-Modellierung
  dateiliste_kindyn="$dateiliste_kindyn
    robot_codegen_definitions/robot_tree_floatb_eulangrpy_definitions.mpl
    robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mpl
    robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
    robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
    robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_linkframe.mpl
    robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
    robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
    robot_codegen_energy/robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
    robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1.mpl
    robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl
  "

  # Initialisiere zusätzliche Maple-Skripte speziell für dieses System (benutzerdefiniert)
  # Mit Basis-Methode "eulangrpy"
  addlistfile=$repo_pfad/robot_codegen_additional/scripts/${robot_name}_maple_additional_worksheet_list_eulangrpy
  if [ -f $addlistfile ]; then
    dateiliste_kindyn="$dateiliste_kindyn `cat $addlistfile`"
  fi;
fi; # floatb

# Alle Maple-Dateien der Reihe nach ausführen
cd /opt/maple18/bin
for mpldat in $dateiliste_kindyn
do
  mpldat_full=$repo_pfad/$mpldat
  filename="${mpldat_full##*/}"                      # Strip longest match of */ from start
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Maple im Kommandozeilenmodus starten (vorher ins richtige Verzeichnis wechseln)
  echo "Starte Maple-Skript $filename"
  nice -n 10 ./maple <<< "currentdir(\"$dir\"): read \"$filename\";"
done

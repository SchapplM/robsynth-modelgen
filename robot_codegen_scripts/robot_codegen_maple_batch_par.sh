#!/bin/bash -e
# Starte alle Maple-Skripte in der richtigen Reihenfolge
# Führe so viele Berechnungen wie möglich parallel aus
#
# Argumente:
# --fixb_only
#   Nur Berechnung der Fixed-Base Funktionen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad


# Standard-Einstellungen
CG_FIXBONLY=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    --fixb_only)
    CG_FIXBONLY=1
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done


# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh


if [ "$CG_FIXBONLY" == "1" ]; then
  # Berechne alles nur für fixed-base Modellierung (dafür reicht die Methode "twist")
  basemethodenames=( twist )
else
  # Berechne auch die floating-base Modellierung
  basemethodenames=( twist eulangrpy )
fi;

for basemeth in "${basemethodenames[@]}"
do
	dateiliste_kin="
		  robot_codegen_definitions/robot_tree_floatb_${basemeth}_definitions.mpl
      robot_codegen_constraints/${robot_name}_kinematic_constraints.mpl
			robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mpl
			robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
	"
	dateiliste_mdhvel="
	    robot_codegen_kinematics/robot_tree_velocity_mdh_angles.mpl
  "
	dateiliste_vel="
	    robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
	    robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_linkframe.mpl
	"
	dateiliste_en="
      robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
      robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
      robot_codegen_energy/robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
	"
	dateiliste_dyn="
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1_corvec.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1_cormat.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1_grav.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1_inertia.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1_inertiaD.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1_invdyn.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2_corvec.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2_cormat.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2_grav.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2_inertia.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2_inertiaD.mpl
		  robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2_invdyn.mpl
	"

  # Zusätzliche Maple-Skripte speziell für dieses System (benutzerdefiniert)
  addlistfile=$repo_pfad/robot_codegen_additional/scripts/${robot_name}_maple_additional_worksheet_list
  if [ -f $addlistfile ]; then
    dateiliste_add="$dateiliste_kindyn `cat $addlistfile`"
  else
    dateiliste_add=""
  fi;


  # Alle Arbeitsblätter parallel ausführen, wo dies möglich ist
  cd /opt/maple18/bin
  for wskin in ${dateiliste_kin[@]}
  do
    mpldat_full=$repo_pfad/$wskin
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    ./maple -q <<< "currentdir(\"$dir\"): read \"$filename\";"
  done
  echo "FERTIG mit Kinematik für ${basemeth}"
  for wsvelm in ${dateiliste_mdhvel[@]}
  do
    mpldat_full=$repo_pfad/$wsvelm
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    ./maple -q <<< "currentdir(\"$dir\"): read \"$filename\";"
  done
  for wsvel in ${dateiliste_vel[@]}
  do
    mpldat_full=$repo_pfad/$wsvel
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    ./maple -q  <<< "currentdir(\"$dir\"): read \"$filename\";" &
  done
  wait
  echo "FERTIG mit Geschwindigkeit für ${basemeth}"

  for wsen in ${dateiliste_en[@]}
  do
    mpldat_full=$repo_pfad/$wsen
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    ./maple -q  <<< "currentdir(\"$dir\"): read \"$filename\";" &
  done
  wait
  echo "FERTIG mit Energie für ${basemeth}"

  for wsdyn in ${dateiliste_dyn[@]}
  do
    mpldat_full=$repo_pfad/$wsdyn
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    ./maple -q  <<< "currentdir(\"$dir\"): read \"$filename\";" &
  done
  wait
  echo "FERTIG mit Dynamik für ${basemeth}"
done


# Jacobi-Matrizen
dateiliste_jac=""
for (( ib=1; ib<=$robot_NL; ib++ ))
do
  dateiliste_jac="$dateiliste_jac
        /robot_codegen_kinematics/robot_tree_rotmat_jacobian_baseframe_body${ib}.mpl
  "
done
for wsjac in ${dateiliste_jac[@]}
do
  mpldat_full=$repo_pfad/$wsjac
  filename="${mpldat_full##*/}"
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
  ./maple -q  <<< "currentdir(\"$dir\"): read \"$filename\";" &
done
wait
echo "FERTIG mit Jacobi-Matrizen"

dateiliste_plin="
  /robot_codegen_energy/robot_chain_fixb_rotmat_energy_regressor.mpl
  /robot_codegen_dynamics/robot_chain_fixb_rotmat_dynamics_regressor.mpl
"
for wsplin in ${dateiliste_plin[@]}
do
  mpldat_full=$repo_pfad/$wsplin
  filename="${mpldat_full##*/}"
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
  ./maple -q  <<< "currentdir(\"$dir\"): read \"$filename\";"
done

if [ -f $addlistfile ]; then
  for wsadd in ${dateiliste_add[@]}
  do
    mpldat_full=$repo_pfad/$wsadd
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    ./maple -q  <<< "currentdir(\"$dir\"): read \"$filename\";" &
  done
  echo "Zusätzlichen Dateien gestartet"
fi;

wait
echo "Alle Matlab-Funktionen exportiert"

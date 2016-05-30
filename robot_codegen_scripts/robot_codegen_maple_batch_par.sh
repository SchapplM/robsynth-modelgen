#!/bin/bash 
# Starte alle Maple-Skripte in der richtigen Reihenfolge
# Führe so viele Berechnungen wie möglich parallel aus
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad

# Dynamik-Skripte für Parametersätze 1 und 2 vorbereiten
cp $repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par12.mpl $repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1.mpl
cp $repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par12.mpl $repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl

# Erstelle einzelne Arbeitsblätter für jeden Teil der inversen Dynamik
codeexportswitches=( corvec cormat grav inertia inertiaD invdyn )
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for ces in "${codeexportswitches[@]}"
  do
    mpldat=$repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par${dynpar}_${ces}.mpl
    cp $repo_pfad/robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par${dynpar}.mpl $mpldat
		# deaktiviere jede Code-Exportierung
		for ces2 in "${codeexportswitches[@]}"
		do
		  sed -i "s/codeexport_${ces2} := true:/codeexport_${ces2} := false:/g" $mpldat
    done
    # Aktivierung der gewünschten Code-Exportierung
		sed -i "s/codeexport_${ces} := false:/codeexport_${ces} := true:/g" $mpldat
  done
done

basemethodenames=( twist eulangrpy )
for basemeth in "${basemethodenames[@]}"
do
	dateiliste_kin="
		  robot_codegen_definitions/robot_tree_floatb_${basemeth}_definitions.mpl
			robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mpl
			robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
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

  # Alle Arbeitsblätter parallel ausführen, wo dies möglich ist
  cd /opt/maple18/bin
  for wskin in ${dateiliste_kin[@]}
  do
    # echo $wskin
    mpldat_full=$repo_pfad/$wskin
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

echo "Alle Matlab-Funktionen exportiert"
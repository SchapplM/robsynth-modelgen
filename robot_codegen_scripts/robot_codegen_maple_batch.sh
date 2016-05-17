#!/bin/bash 
# Starte alle Maple-Skripte nacheinander in der richtigen Reihenfolge
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

# Liste mit Maple-Skripten in der richtigen Reihenfolge
dateiliste_kindyn="
    /robot_codegen_definitions/robot_tree_floatb_twist_definitions.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_linkframe.mpl
    /robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
    /robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
    /robot_codegen_energy/robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
    /robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1.mpl
    /robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl
    /robot_codegen_definitions/robot_tree_floatb_eulangrpy_definitions.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
    /robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_linkframe.mpl
    /robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
    /robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
    /robot_codegen_energy/robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
    /robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par1.mpl
    /robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl
    /robot_codegen_energy/robot_chain_fixb_rotmat_energy_regressor.mpl
    /robot_codegen_dynamics/robot_chain_fixb_rotmat_dynamics_regressor.mpl
"

# Alle Maple-Dateien der Reihe nach ausführen
cd /opt/maple18/bin
for mpldat in $dateiliste_kindyn
do
  mpldat_full=$repo_pfad/$mpldat
  filename="${mpldat_full##*/}"                      # Strip longest match of */ from start
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Maple im Kommandozeilenmodus starten (vorher ins richtige Verzeichnis wechseln)
  ./maple <<< "currentdir(\"$dir\"): read \"$filename\";"

done


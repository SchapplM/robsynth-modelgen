#!/bin/bash -e
# Vorbereitung der Maple-Skripte für die automatische Verarbeitung
# Die Skripte werden teilweise mit unterschiedlichen Parametern versehen und neu gespeichert.
#
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad


source $repo_pfad/robot_codegen_definitions/robot_env.sh


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


# Jacobi-Matrix-Skripte vorbereiten, so dass die Jacobi-Matrix für alle Körper generiert wird.
for (( ib=1; ib<=$robot_NL; ib++ ))
do
  mpldat=$repo_pfad/robot_codegen_kinematics/robot_tree_rotmat_jacobian_baseframe_body${ib}.mpl
  cp $repo_pfad/robot_codegen_kinematics/robot_tree_rotmat_jacobian_baseframe.mpl $mpldat
  # Die Jacobi-Matrix für den aktuellen Körper erstellen
  sed -i "s/LIJAC:=NL:/LIJAC:=${ib}:/g" $mpldat
done


echo "Maple-Skripte zur Stapelverarbeitung vorbereitet."

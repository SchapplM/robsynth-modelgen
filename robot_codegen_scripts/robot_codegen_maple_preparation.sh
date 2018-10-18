#!/bin/bash -ex
# Vorbereitung der Maple-Skripte für die automatische Verarbeitung
# Die Skripte werden teilweise mit unterschiedlichen Parametern versehen und neu gespeichert.
#
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..

# Standard-Einstellungen
CG_MINIMAL=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    --minimal)
    CG_MINIMAL=1
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Arbeitsverzeichnis leeren (damit alte Dateiversionen nicht versehentlich gestartet werden). 
rm -rf $repo_pfad/workdir
mkdir -p $repo_pfad/workdir/tmp

# Alle mpl-Dateien in Arbeitsverzeichnis kopieren
for mpldat in `find $repo_pfad -name "*.mpl"`; do
  mpldat_full=$repo_pfad/$mpldat
  filename="${mpldat_full##*/}"
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename
  if [[ "$dir" == *workdir ]]; then
    # ignoriere gefundene mpl-Dateien, die bereits im Zielverzeichnis sind
    continue
  fi;
  filename="${mpldat_full##*/}"
  cp $mpldat $repo_pfad/workdir/$filename
done

# Dateien im Arbeitsverzeichnis bearbeiten: 
# Debug-Ausgabe in allen Skripten entfernen
if [ "$CG_MINIMAL" == "1" ]; then
  for mpldat in `find $repo_pfad/workdir -name "*.mpl"`; do
      # TODO: Leerzeichen behandeln
		  sed -i "s/codegen_debug := true:/codegen_debug := false:/g" $mpldat
  done
fi

# Dateien im Arbeitsverzeichnis bearbeiten:
# Code-Generierung für einige Skripte entfernen
# Ziel: Nur den Matlab-Code exportieren, den man auch wirklich für die Dynamik benötigt.
if [ "$CG_MINIMAL" == "1" ]; then
  noexportlist="
    robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
    robot_tree_velocity_mdh_angles.mpl
    robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
    robot_tree_floatb_rotmat_velocity_linkframe.mpl
    robot_tree_rotmat_jacobian_baseframe.mpl
    robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
    robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
    robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
    robot_chain_fixb_rotmat_energy_regressor.mpl
    robot_chain_floatb_rotmat_energy_regressor.mpl
    robot_tree_base_parameter_transformations.mpl
    ${robot_name}_kinematic_constraints.mpl
    robot_kinematic_constraints_calculations.mpl
  "
  for f in $noexportlist; do
      if [ ! -f $repo_pfad/workdir/$f ]; then
        continue # Datei existiert nicht.
      fi 
      mpldat=$repo_pfad/workdir/$f
      # TODO: Leerzeichen behandeln
		  sed -i "s/codegen_act := true:/codegen_act := false:/g" $mpldat
  done
fi


# Lagrange-Skripte für Parametersätze 1 und 2 vorbereiten
cp $repo_pfad/workdir/robot_tree_floatb_rotmat_lagrange_worldframe_par12.mpl $repo_pfad/workdir/robot_tree_floatb_rotmat_lagrange_worldframe_par1.mpl
cp $repo_pfad/workdir/robot_tree_floatb_rotmat_lagrange_worldframe_par12.mpl $repo_pfad/workdir/robot_tree_floatb_rotmat_lagrange_worldframe_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_tree_floatb_rotmat_lagrange_worldframe_par2.mpl

# Dynamik-Skripte für Parametersätze 1 und 2 vorbereiten
cp $repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par12.mpl $repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par1.mpl
cp $repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par12.mpl $repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par2.mpl

# Erstelle einzelne Arbeitsblätter für jeden Teil der inversen Dynamik
codeexportswitches=( corvec cormat grav inertia inertiaD invdyn )
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for ces in "${codeexportswitches[@]}"
  do
    mpldat=$repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par${dynpar}_${ces}.mpl
    cp $repo_pfad/workdir/robot_tree_floatb_rotmat_dynamics_worldframe_par${dynpar}.mpl $mpldat
		# deaktiviere jede Code-Exportierung
		for ces2 in "${codeexportswitches[@]}"
		do
		  sed -i "s/codeexport_${ces2} := true:/codeexport_${ces2} := false:/g" $mpldat
    done
    # Aktivierung der gewünschten Code-Exportierung
		sed -i "s/codeexport_${ces} := false:/codeexport_${ces} := true:/g" $mpldat
  done
done

# Regressor-Berechnung für Minimalparameter und Parametersatz 2 vorbereiten
cp $repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor.mpl $repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor_minpar.mpl
cp $repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor.mpl $repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor_pv2.mpl
sed -i "s/regressor_modus := \"regressor_minpar\":/regressor_modus := \"regressor\":/g" $repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor_pv2.mpl

# Erstelle einzelne Arbeitsblätter für jeden Teil der inversen Dynamik als Regressor
codeexportswitches=( corvec cormat grav inertia inertiaD invdyn )
for rm in "minpar" "pv2"
do
  for ces in "${codeexportswitches[@]}"
  do
    mpldat=$repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor_${rm}_${ces}.mpl
    cp $repo_pfad/workdir/robot_chain_floatb_rotmat_dynamics_regressor_${rm}.mpl $mpldat
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
for (( ib=0; ib<$robot_NL; ib++ ))
do
  mpldat=$repo_pfad/workdir/robot_tree_rotmat_jacobian_baseframe_body${ib}.mpl
  cp $repo_pfad/workdir/robot_tree_rotmat_jacobian_baseframe.mpl $mpldat
  # Die Jacobi-Matrix für den aktuellen Körper erstellen
  sed -i "s/LIJAC:=NL-1:/LIJAC:=${ib}:/g" $mpldat
done

echo "Maple-Skripte zur Stapelverarbeitung vorbereitet."

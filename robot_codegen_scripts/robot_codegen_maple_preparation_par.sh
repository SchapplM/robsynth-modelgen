#!/bin/bash -e
# Vorbereitung der Maple-Skripte für die automatische Verarbeitung
# Die Skripte werden teilweise mit unterschiedlichen Parametern versehen und neu gespeichert.
#
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

repo_pfad=$(pwd)/..

source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Roboterdefinition kopieren
cp "$repo_pfad/robot_codegen_definitions/robot_env_par" "$repo_pfad/codeexport/$robot_name/tmp"

# Alle mpl-Dateien im Ordner robot_codegen_parallel in Arbeitsverzeichnis kopieren
# Dieser Schritt wird bereits durch robot_codegen_maple_preparation.sh getan
# Neu-Durchführung, falls das Skript für serielle Roboter nicht ausgeführt wurde.
for mpldat_full in `find $repo_pfad/robot_codegen_parallel -name "*.mpl"`; do
  filename="${mpldat_full##*/}"
  cp $mpldat_full $repo_pfad/workdir/$filename
done

# Dateien im Arbeitsverzeichnis bearbeiten:
# Debug-Ausgabe in allen Skripten entfernen
if [ "$CG_MINIMAL" == "1" ]; then
  for mpldat in `find $repo_pfad/workdir -name "*.mpl"`; do
      # TODO: Leerzeichen behandeln
      sed -i "s/codegen_debug := true:/codegen_debug := false:/g" $mpldat
  done
fi

# Parallele Dynamik-Skripte für Parametersätze 1 und 2 vorbereiten
# ... für die Plattform
cp $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics.mpl $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics_par1.mpl
cp $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics.mpl $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics_par2.mpl
# ... für die Gesamt-Dynamik
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_par1.mpl
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_par2.mpl
# ... Export-Arbeitsblatt
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export_par1.mpl
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export_par2.mpl

# Erstelle einzelne Arbeitsblätter für jeden Teil der inversen Dynamik
codeexportswitches=( corvec grav inertia invdyn )
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for ces in "${codeexportswitches[@]}"
  do
    mpldat=$repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export_par${dynpar}_${ces}.mpl
    cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_export_par${dynpar}.mpl $mpldat
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
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_minpar.mpl
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_pv2.mpl
sed -i "s/regressor_modus := \"regressor_minpar\":/regressor_modus := \"regressor\":/g" $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_pv2.mpl
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export_minpar.mpl
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export_pv2.mpl
sed -i "s/regressor_modus := \"regressor_minpar\":/regressor_modus := \"regressor\":/g" $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export_pv2.mpl

# Erstelle einzelne Export-Arbeitsblätter für jeden Teil der inversen Dynamik (Regressorform)
codeexportswitches=( corvec grav inertia invdyn )
regmodeswitches="minpar pv2"
for rms in ${regmodeswitches}; do
  for ces in "${codeexportswitches[@]}"
  do
    mpldat=$repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export_${rms}_${ces}.mpl
    cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_regressor_export_${rms}.mpl $mpldat
    # deaktiviere jede Code-Exportierung
    for ces2 in "${codeexportswitches[@]}"
    do
      sed -i "s/codeexport_${ces2} := true:/codeexport_${ces2} := false:/g" $mpldat
    done
    # Aktivierung der gewünschten Code-Exportierung
    sed -i "s/codeexport_${ces} := false:/codeexport_${ces} := true:/g" $mpldat
  done
done

echo "Maple-Skripte zur Stapelverarbeitung vorbereitet."

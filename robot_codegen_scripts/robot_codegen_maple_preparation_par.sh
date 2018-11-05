#!/bin/bash -ex
# Vorbereitung der Maple-Skripte für die automatische Verarbeitung
# Die Skripte werden teilweise mit unterschiedlichen Parametern versehen und neu gespeichert.
#
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..

source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Alle mpl-Dateien in Arbeitsverzeichnis kopieren
for mpldat in `find $repo_pfad/robot_codegen_parallel -name "*.mpl"`; do
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

# Parallele Dynamik-Skripte für Parametersätze 1 und 2 vorbereiten
cp $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics.mpl $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics_par1.mpl
cp $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics.mpl $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_para_plattform_rotmat_dynamics_par2.mpl

cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_par1.mpl
cp $repo_pfad/workdir/robot_para_rotmat_projection_dynamics.mpl $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_par2.mpl
sed -i "s/codegen_dynpar := 1:/codegen_dynpar := 2:/g" $repo_pfad/workdir/robot_para_rotmat_projection_dynamics_par2.mpl

echo "Maple-Skripte zur Stapelverarbeitung vorbereitet."

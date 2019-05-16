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

source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Roboterdefinition kopieren
# cp "$repo_pfad/robot_codegen_definitions/robot_env_ic" "$repo_pfad/codeexport/$robot_name/tmp"

# Alle mpl-Dateien im Ordner robot_codegen_constraints in Arbeitsverzeichnis kopieren
# Dieser Schritt wird bereits durch robot_codegen_maple_preparation.sh getan
# Neu-Durchführung, falls das Skript für serielle Roboter nicht ausgeführt wurde.
for mpldat_full in `find $repo_pfad/robot_codegen_constraints -name "*.mpl"`; do
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

echo "Maple-Skripte zur Stapelverarbeitung vorbereitet."

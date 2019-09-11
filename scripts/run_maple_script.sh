#!/bin/bash -e
# Führe ein Maple-Skript (mpl) mit Maple in der Eingabeaufforderung aus
# Argument: Pfad zur mpl-Datei
# Mögliche Umgebungsvariablen:
# * MAPLE_BIN: Pfad zur Exe-Datei des Konsolen-Maple (wird mit export... gesetzt)

# Moritz Schappler, schappler@irt.uni-hannover.de, 2018-03
# (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

# Pfad zur mpl-Datei
mpl_filepath=$1
mpl_dir=${mpl_filepath%/*}     # Verzeichnis
mpl_file="${mpl_filepath##*/}" # Dateiname

# Umgebungsvariable prüfen: Pfad zu Maple
if [ "$MAPLE_BIN" == "" ]; then
  # Standard-Pfad unter Linux
  MAPLE_BIN=/opt/maple2019/bin/maple
fi
maple_path=${MAPLE_BIN%/*}   # Verzeichnis
maple_exc="${MAPLE_BIN##*/}" # Dateiname

# In Verzeichnis des Skriptes wechseln und Maple dort starten mit den Skript als Argument.
# Maple startet dann in diesem Verzeichnis und kann das Skript ausführen (mit niedrigerer Prozessor-Prio.).
# Diese Vorgehensweise ist notwendig, wenn Windows-Maple über ein Bash-Skript aus dem Windows-Subsystem für Linux ausgeführt wird.
pwd_alt=$(pwd)
cd "$mpl_dir"
nice -n 10 "$maple_path/$maple_exc" -q  <<< "read \"$mpl_file\";"
cd $pwd_alt

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

# Pfad zu dieser Datei (run_maple_script.sh) führt zum Repo
repo_dir=${0%/*}/..
# Datei zum Speichern des absolten Pfades zur Maple-Binary
# Bei WSL wird der Pfad im WSL-Dateisystem angegeben
mplbin_file=$repo_dir/MAPLE_BIN.env

# Umgebungsvariable prüfen: Pfad zu Maple
if [ "$MAPLE_BIN" == "" ]; then
  if [ -f $mplbin_file ]; then
    MAPLE_BIN=`cat $mplbin_file`
  else
    echo "Pfad zur Maple-Binary weder in Umgebungsvariable MAPLE_BIN, noch in Linkdatei $mplbin_file gegeben"
    exit 1
  fi
fi

if [ ! -f "$MAPLE_BIN" ]; then
  echo "Maple-Programmdatei aus Umgebungsvariable MAPLE_BIN existiert nicht: $MAPLE_BIN"
  exit 1
fi

maple_path=${MAPLE_BIN%/*}   # Verzeichnis
maple_exc="${MAPLE_BIN##*/}" # Dateiname

# In Verzeichnis des Skriptes wechseln und Maple dort starten mit den Skript als Argument.
# Maple startet dann in diesem Verzeichnis und kann das Skript ausführen (mit niedrigerer Prozessor-Prio.).
# Diese Vorgehensweise ist notwendig, wenn Windows-Maple über ein Bash-Skript aus dem Windows-Subsystem für Linux ausgeführt wird.
pwd_alt=$(pwd)
cd "$mpl_dir"
echo "Run \"$mpl_file\" using $MAPLE_BIN"
nice -n 10 "$maple_path/$maple_exc" -q  <<< "read \"$mpl_file\";"
cd $pwd_alt

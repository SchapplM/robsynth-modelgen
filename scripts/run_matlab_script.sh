#!/bin/bash -e
# Führe ein Matlab-Skript (m) mit Matlab in der Eingabeaufforderung aus
# Argument: Pfad zur m-Datei
# Mögliche Umgebungsvariablen:
# * MATLAB_BIN: Pfad zur Exe-Datei des Konsolen-Matlab (wird mit export... gesetzt)

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2024-05
# (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

# Pfad zur m-Datei
m_filepath=$1
m_dir=${m_filepath%/*}     # Verzeichnis
m_file="${m_filepath##*/}" # Dateiname

# Finde heraus, ob wir uns unter WSL befinden und wandle den Pfad um
if [[ $(grep -i Microsoft /proc/version) ]]; then
  m_filepath=`wslpath -w $m_filepath`
  echo "Bash läuft auf WSL. Skript-Pfad: $m_filepath"
fi

# Pfad zu dieser Datei (run_matlab_script.sh) führt zum Repo
repo_dir=${0%/*}/..
# Datei zum Speichern des absoluten Pfades zur MATLAB-Binary
# Bei WSL wird der Pfad im WSL-Dateisystem angegeben
mbin_file=$repo_dir/MATLAB_BIN.env

# Prüfe, ob Matlab normal installiert ist
# (Unter Ubuntu mit Paket matlab-support)
if [ "$MATLAB_BIN" == "" ]; then
  MATLAB_BIN=`which matlab`
fi

# Umgebungsvariable prüfen: Pfad zu MATLAB
if [ "$MATLAB_BIN" == "" ]; then
  if [ -f $mbin_file ]; then
    MATLAB_BIN=`cat $mbin_file`
  else
    echo "Pfad zur MATLAB-Binary weder in Umgebungsvariable MATLAB_BIN, noch in Linkdatei $mbin_file gegeben"
    exit 1
  fi
fi

if [ ! -f "$MATLAB_BIN" ]; then
  echo "MATLAB-Programmdatei aus Umgebungsvariable MATLAB_BIN existiert nicht: $MATLAB_BIN"
  exit 1
fi

MATLAB_path=${MATLAB_BIN%/*}   # Verzeichnis
MATLAB_exc="${MATLAB_BIN##*/}" # Dateiname

# MATLAB starten mit den Skript als Argument (mit niedrigerer Prozessor-Prio.).
pwd_alt=$(pwd)
echo "Run \"$m_file\" using $MATLAB_BIN"
nice -n 10 "$MATLAB_path/$MATLAB_exc" -nodesktop -nosplash -useStartupFolderPref -r "run('$m_filepath');quit;"
cd $pwd_alt

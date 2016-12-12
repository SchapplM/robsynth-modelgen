#!/bin/bash -e 
# Nachbearbeitung aller automatisch generierter Matlab-Dateien (Code, keine Funktionen)
# Bearbeitet das Verzeichnis codeexport in diesem Repo
#
# Argument: Verzeichnis der m-Dateien


# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

if [ "$1" == "" ] || [ ! -d "$1" ]; then
  echo "Eingabeargument \"$1\" ist kein gültiges Verzeichnis."
  exit 1
fi;

# Alle Matlab-Dateien im Ordner codeexport korrigieren, falls notwendig
for f in $(find $1 -maxdepth 1 -name "*_matlab.m")
do
  mtime_old=`stat $f --printf '%Y'`
  ./robot_codegen_matlabcode_postprocess.sh $f
  mtime_new=`stat $f --printf '%Y'`
  if [ $mtime_old -ne $mtime_new ]; then
    echo "Postprocessed $f"
  fi;
done


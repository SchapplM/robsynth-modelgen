#!/bin/bash -e
# Speichere die Aktuelle Roboterkonfiguration für hybride Roboter als Bash-Variable in einer Datei
# Code-Schnipsel enthalten Variablenzuweisungen
#
# Argumente: 
# "quiet": Keine Textausgabe in Konsole.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

# Öffne die Umgebungsvariable und speichere die Informationen als shell-variable
repo_pfad=$(pwd)/..
robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env_IC

if [ ! -f "$robot_env_pfad" ]; then
  echo "Keine Roboterdefinition \"$robot_env_pfad\" für hybriden Roboter mit impliziten Zwangsbedingungen gefunden"
  exit 1
fi;

# Lese die Informationen aus der Eingabe-Maple-Datei
robot_name_OL=`grep "robot_name_OL := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`

echo "robot_name=\"$robot_name\"" > $robot_env_pfad.sh
echo "robot_name_OL=\"$robot_name_OL\"" >> $robot_env_pfad.sh

if [ -d "$repo_pfad/codeexport/${robot_name}/" ]; then
	cp $repo_pfad/robot_codegen_definitions/robot_env_IC $repo_pfad/codeexport/${robot_name}/tmp/
	cp $repo_pfad/robot_codegen_definitions/robot_env_IC.sh $repo_pfad/codeexport/${robot_name}/tmp/
fi;

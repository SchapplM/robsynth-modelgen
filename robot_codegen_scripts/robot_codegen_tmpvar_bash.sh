#!/bin/bash 
# Speichere die Aktuelle Roboterkonfiguration als Bash-Variable in einer Datei
# Code-Schnipsel enthalten Variablenzuweisungen
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

# Öffne die Umgebungsvariable und speichere die Informationen als shell-variable
repo_pfad=$(pwd)/..
robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env

# Lese die Informationen aus der Maple-Datei
robot_NJ=`grep "NJ := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`

echo "robot_NJ=$robot_NJ"
echo "robot_name=$robot_name"

# Speichere die Daten als Shell-Variablen. Die Variablen werden von anderen Skripten mit `robot_codegen_definitions/robot_env.sh` eingebunden.
echo "robot_NJ=$robot_NJ" > $robot_env_pfad.sh
echo "robot_name=$robot_name" >> $robot_env_pfad.sh

# Lese weitere Informationen aus der generierten Definitionsdatei
robot_def_pfad=$repo_pfad/codeexport/${robot_name}_tree_floatb_twist_definitions
robot_NL=`grep "NL := " $robot_def_pfad | tail -1 | sed 's/.*= \(.*\);/\1/'`
echo "robot_NL=$robot_NL" >> $robot_env_pfad.sh

echo "robot_NL=$robot_NL"

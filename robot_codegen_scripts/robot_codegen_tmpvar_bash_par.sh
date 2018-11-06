#!/bin/bash -e
# Speichere die Aktuelle Roboterkonfiguration als Bash-Variable in einer Datei
# Code-Schnipsel enthalten Variablenzuweisungen
#
# Argumente: 
# "quiet": Keine Textausgabe in Konsole.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

# Öffne die Umgebungsvariable und speichere die Informationen als shell-variable
repo_pfad=$(pwd)/..
robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env_par

if [ ! -f "$robot_env_pfad" ]; then
  echo "Keine Roboterdefinition \"$robot_env_pfad\" für parallelen Roboter gefunden"
  exit 1
fi;

# Lese die Informationen aus der Eingabe-Maple-Datei
robot_leg_name=`grep "leg_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`

#parallel_robot=`grep "parallel := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
parallal_leg_name=`grep "leg_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
parallel_NLEGS=`grep "N_LEGS := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`

# Variablen für parallelroboter
NQJ_parallel_pfad=$repo_pfad/codeexport/$robot_name/tmp/var_parallel.m
parallel_NX=`grep "unknown(1,1) = " $NQJ_parallel_pfad | tail -1 | sed 's/.*= \(.*\);/\1/'`
parallel_NQJ_leg=`grep "unknown(2,1) = " $NQJ_parallel_pfad | tail -1 | sed 's/.*= \(.*\);/\1/'`
parallel_angles_leg=`grep "unknown(3,1) = " $NQJ_parallel_pfad | tail -1 | sed 's/.*= \(.*\);/\1/'`
robot_system_q=$(( parallel_NQJ_leg * parallel_NLEGS ))

echo "parallel_NX=$parallel_NX" > $robot_env_pfad.sh
echo "parallel_NQJ_leg=$parallel_NQJ_leg" >> $robot_env_pfad.sh
echo "parallel_angles_leg=$parallel_angles_leg" >> $robot_env_pfad.sh
echo "robot_system_q=$robot_system_q" >> $robot_env_pfad.sh
echo "parallel_NLEGS=$parallel_NLEGS" >> $robot_env_pfad.sh
echo "robot_name=\"$robot_name\"" >> $robot_env_pfad.sh
echo "robot_leg_name=\"$robot_leg_name\"" >> $robot_env_pfad.sh

if [ -d "$repo_pfad/codeexport/${robot_name}/" ]; then
	cp $repo_pfad/robot_codegen_definitions/robot_env_par $repo_pfad/codeexport/${robot_name}/tmp/
	cp $repo_pfad/robot_codegen_definitions/robot_env_par.sh $repo_pfad/codeexport/${robot_name}/tmp/
fi;
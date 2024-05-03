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
robot_name_DE=`grep "robot_name_DE := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name_TE=`grep "robot_name_TE := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name_OL=`grep "robot_name_OL := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`

# Variablen für Implizit definierten Hybrid-Roboter (aus exportiertem Code)
# Berücksichtige CR-Zeichen des WSL über [\r]*
NAJ_pfad=$repo_pfad/codeexport/$robot_name/tmp/NAJ_ic_matlab.m
if [ -f $NAJ_pfad ]; then
	robot_NAJ=`grep "t1 = " $NAJ_pfad | tail -1 | sed 's/.*= \(.*\);[\r]*/\1/'`
else
	robot_NAJ="NOTDEFINED"
fi

echo "robot_name=\"$robot_name\"" > $robot_env_pfad.sh
echo "robot_name_OL=\"$robot_name_OL\"" >> $robot_env_pfad.sh
echo "robot_name_TE=\"$robot_name_TE\"" >> $robot_env_pfad.sh
echo "robot_name_DE=\"$robot_name_DE\"" >> $robot_env_pfad.sh
echo "robot_NAJ=\"$robot_NAJ\"" >> $robot_env_pfad.sh

if [ -d "$repo_pfad/codeexport/${robot_name}/" ]; then
	cp $repo_pfad/robot_codegen_definitions/robot_env_IC $repo_pfad/codeexport/${robot_name}/tmp/
	cp $repo_pfad/robot_codegen_definitions/robot_env_IC.sh $repo_pfad/codeexport/${robot_name}/tmp/
fi;

# Dimension der Kinematikparameter (sind jetzt vielleicht andere als bei
# serieller Hauptstruktur (durch zusätzliche implizite ZB)
# Siehe gleichen Code in Datei robot_codegen_tmpvar_bash.sh
robot_KP_pfad=$repo_pfad/codeexport/${robot_name}/tmp/parameter_kin
if [ -f $robot_KP_pfad ]; then
  sed -i 's/\r//g' $robot_KP_pfad # Zeilenenden
  robot_NKP=`sed -n -e 's/pkin := Matrix(\([[:alnum:]]\+\), 1.*/\1/p' $robot_KP_pfad`
  robot_KP=`tr -d "\n" < $robot_KP_pfad | sed -n -e 's/.*pkin := Matrix(\([[:alnum:]]\+\), 1, \[\[\(.*\)\]\]);/\2/p' | sed 's/,/ /g' | sed 's/\[//g' | sed 's/\]//g'`
else
  robot_KP="UNDEFINED"
fi
if [[ "$robot_NKP" == "0" ]]; then
  # Falls keine Kinematikparameter notwendig sind, definiere Dummy-Parameter
  # Ansonsten gibt es Probleme in Simulink-Blöcken wegen Signal mit 0-Dimension
  robot_NKP=1
  robot_KP="dummy"
fi;


echo "robot_NKP=$robot_NKP" >> $robot_env_pfad.sh
echo "robot_KP=\"$robot_KP\"" >> $robot_env_pfad.sh
if [ ! "$1" == "quiet" ]; then
  echo "robot_NKP=$robot_NKP"
fi

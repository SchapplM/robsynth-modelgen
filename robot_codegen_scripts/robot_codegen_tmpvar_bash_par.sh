#!/bin/bash -e
# Speichere die Aktuelle Roboterkonfiguration als Bash-Variable in einer Datei
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
robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env_par

if [ ! -f "$robot_env_pfad" ]; then
  echo "Keine Roboterdefinition \"$robot_env_pfad\" für parallelen Roboter gefunden"
  exit 1
fi;

# Lese die Informationen aus der Eingabe-Maple-Datei
robot_leg_name=`grep "leg_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`

#parallel_robot=`grep "parallel := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
parallel_NLEGS=`grep "N_LEGS := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`

# Variablen für Parallelroboter (aus exportiertem Code)
NQJ_parallel_pfad=$repo_pfad/codeexport/$robot_name/tmp/var_parallel.m
if [ -f $NQJ_parallel_pfad ]; then
	parallel_NX=`grep "unknown(1,1) = " $NQJ_parallel_pfad | tail -1 | sed 's/.*= \(.*\);[\r]*/\1/'`
	parallel_NQJ_leg=`grep "unknown(2,1) = " $NQJ_parallel_pfad | tail -1 | sed 's/.*= \(.*\);[\r]*/\1/'`
	parallel_angles_leg=`grep "unknown(3,1) = " $NQJ_parallel_pfad | tail -1 | sed 's/.*= \(.*\);[\r]*/\1/'`
else
	parallel_NX="NOTDEFINED"
	parallel_NQJ_leg="NOTDEFINED"
	parallel_angles_leg="NOTDEFINED"
fi
# Extrahiere den G-Vektor aus der Definitionsdatei (zur Vorgabe eines reduzierten g-Vektors)
robot_def_pfad=$repo_pfad/codeexport/$robot_name/tmp/para_definitions
if [ -f $robot_def_pfad ]; then
  robot_gVec=`grep "g_world := Matrix(3, 1, " $robot_def_pfad | tail -1 | sed 's/.*\[\[\([a-z,0-9]*\)\]\(,\)\[\([a-z,0-9]*\)\]\(,\)\[\([a-z,0-9]*\).*;/\1\2\3,\5/'`
  robot_gVec="$(sed s/[a-z][0-9]/1/g <<<$robot_gVec)"
else
  robot_gVec="UNDEFINED"
fi
robot_system_q=$(( parallel_NQJ_leg * parallel_NLEGS ))

echo "parallel_NX=$parallel_NX" > $robot_env_pfad.sh
echo "parallel_NQJ_leg=$parallel_NQJ_leg" >> $robot_env_pfad.sh
echo "parallel_angles_leg=$parallel_angles_leg" >> $robot_env_pfad.sh
echo "robot_system_q=$robot_system_q" >> $robot_env_pfad.sh
echo "parallel_NLEGS=$parallel_NLEGS" >> $robot_env_pfad.sh
echo "robot_name=\"$robot_name\"" >> $robot_env_pfad.sh
echo "robot_leg_name=\"$robot_leg_name\"" >> $robot_env_pfad.sh
echo "robot_gVec=$robot_gVec" >> $robot_env_pfad.sh

if [ -d "$repo_pfad/codeexport/${robot_name}/" ]; then
	cp $repo_pfad/robot_codegen_definitions/robot_env_par $repo_pfad/codeexport/${robot_name}/tmp/
	cp $repo_pfad/robot_codegen_definitions/robot_env_par.sh $repo_pfad/codeexport/${robot_name}/tmp/
fi;

# Dimension des MPV (aus exportiertem Code)
mpv_para_pfad=$repo_pfad/codeexport/${robot_name}/tmp/RowMinPar_parallel.m
if [ -f $mpv_para_pfad ]; then
  # Ersetze Text links und rechts von der Dimension mit nichts.
  robot_NMPVPARA=`grep "t1 = " $mpv_para_pfad | tail -1 | sed 's/.*= \(.*\);[\r]*/\1/'`
else
  robot_NMPVPARA="NOTDEFINED"
fi
echo "robot_NMPVPARA=$robot_NMPVPARA" >> $robot_env_pfad.sh

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
robot_NQJ=`grep "NQJ := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
robot_NJ=`grep "NJ := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`

echo "robot_NQJ=$robot_NQJ"
echo "robot_NJ=$robot_NJ"
echo "robot_name=$robot_name"

# Speichere die Daten als Shell-Variablen. Die Variablen werden von anderen Skripten mit `robot_codegen_definitions/robot_env.sh` eingebunden.
echo "robot_NQJ=$robot_NQJ" > $robot_env_pfad.sh
echo "robot_NJ=$robot_NJ" >> $robot_env_pfad.sh
echo "robot_name=\"$robot_name\"" >> $robot_env_pfad.sh

# Lese weitere Informationen aus der generierten Definitionsdatei
robot_def_pfad=$repo_pfad/codeexport/${robot_name}_tree_floatb_twist_definitions
robot_NL=`grep "NL := " $robot_def_pfad | tail -1 | sed 's/.*= \(.*\);/\1/'`
echo "robot_NL=$robot_NL" >> $robot_env_pfad.sh
echo "robot_NL=$robot_NL"

# Variablen für die kinematischen Zwangsbedingungen aus der generierten Definitionsdatei
kinconstr_exist=1 # Existenz von Zwangsbedingungen prüfen
if [ `grep "kintmp_s := Matrix(1, 1, \[\[0\]\]);" $robot_def_pfad | wc -l` -eq 1 ]; then
  kinconstr_exist=0
fi
# robot_NKCP: Anzahl der Parameter der kin. ZB
# KCP: Leerzeichengetrennte Liste der Parameter der kinematischen Zwangsbedingungen
# KCPARG: Argument für Matlab-Funktionen
if [ $kinconstr_exist == 1 ]; then
  robot_NKCP=`sed -n -e 's/kintmp_s := Matrix(\([[:digit:]]\+\).*/\1/p' $robot_def_pfad`
  KCP=`tr -d "\n" < $robot_def_pfad | sed -n -e 's/.*kintmp_s := Matrix(\([[:alnum:]]\+\), 1, \[\[\(.*\)\]\]);/\2/p' | sed 's/\],\[/ /g'`
  KCPARG=", kintmp"
else
  robot_NKCP=0
  KCPARG=""
fi;

echo "kinconstr_exist=$kinconstr_exist" >> $robot_env_pfad.sh
echo "robot_NKC=$robot_NKC" >> $robot_env_pfad.sh
echo "KCPARG=\"$KCPARG\"" >> $robot_env_pfad.sh
echo "KCP=\"$KCP\"" >> $robot_env_pfad.sh

# Dimension des MPV (aus exportiertem Code)
mpv_pfad=$repo_pfad/codeexport/${robot_name}_minimal_parameter_vector_maple
if [ -f $mpv_pfad ]; then
  # Ersetze Text links und rechts von der Dimension mit nichts.
  robot_NMPV=`grep "Matrix" $mpv_pfad | tail -1 | sed 's/.*Matrix[(]\(.*\)/\1/' | sed 's/, 1, .*//'`
else
  robot_NMPV="NOTDEFINED"
fi
echo "robot_NMPV=$robot_NMPV" >> $robot_env_pfad.sh
echo "robot_NMPV=$robot_NMPV"



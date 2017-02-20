#!/bin/bash -e
# Speichere die Aktuelle Roboterkonfiguration als Bash-Variable in einer Datei
# Code-Schnipsel enthalten Variablenzuweisungen
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

# Öffne die Umgebungsvariable und speichere die Informationen als shell-variable
repo_pfad=$(pwd)/..
robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env

if [ ! -f "$robot_env_pfad" ]; then
  echo "Keine Roboterdefinition \"$robot_env_pfad\" gefunden"
  exit 1
fi;

# Lese die Informationen aus der Eingabe-Maple-Datei
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
robot_def_pfad=$repo_pfad/codeexport/${robot_name}/tree_floatb_twist_definitions
if [ -f $robot_def_pfad ]; then
  robot_NL=`grep "NL := " $robot_def_pfad | tail -1 | sed 's/.*= \(.*\);/\1/'`
else
  robot_NL="UNDEFINED"
fi;
echo "robot_NL=$robot_NL" >> $robot_env_pfad.sh
echo "robot_NL=$robot_NL"

# Dimension der Kinematikparameter
robot_KP_pfad=$repo_pfad/codeexport/${robot_name}/parameter_kin
if [ -f $robot_KP_pfad ]; then
  robot_NKP=`sed -n -e 's/pkin := Matrix(\([[:alnum:]]\+\), 1.*/\1/p' $robot_KP_pfad`
  robot_KP=`tr -d "\n" < $robot_KP_pfad | sed -n -e 's/.*pkin := Matrix(\([[:alnum:]]\+\), 1, \[\[\(.*\)\]\]);/\2/p' | sed 's/,/ /g' | sed 's/\[//g' | sed 's/\]//g'`
else
  robot_KP="UNDEFINED"
fi
echo "robot_NKP=$robot_NKP" >> $robot_env_pfad.sh
echo "robot_KP=\"$robot_KP\"" >> $robot_env_pfad.sh
echo "robot_NKP=$robot_NKP"

# Variablen für die kinematischen Zwangsbedingungen aus der generierten Definitionsdatei
robot_kinconstr_exist=1 # Existenz von Zwangsbedingungen prüfen (Vorgabe durch Benutzereingabe)
if [ -f $robot_def_pfad ]; then
  if [ `grep "kintmp_s := Matrix(1, 1, \[\[0\]\]);" $robot_def_pfad | wc -l` -eq 1 ]; then
    robot_kinconstr_exist=0
  fi;
else
  robot_kinconstr_exist=0
fi;

robot_KCsymb_pfad=$repo_pfad/codeexport/${robot_name}/kinematic_constraints_symbols_list_maple
# robot_NKCP: Anzahl der Parameter der kin. ZB
# KCP: Leerzeichengetrennte Liste der Parameter der kinematischen Zwangsbedingungen
if [ $robot_kinconstr_exist == 1 ]; then
  # Vom Benutzer sind Symbole für kinematisch Zwangsbedingungen in den MDH-Parametern vorgegeben worden
  # Suche die Liste der Ausdrücke, die von Maple dazu generiert wurden
  if [ -f $robot_KCsymb_pfad ]; then
    robot_NKCP=`sed -n -e 's/kc_symbols := Matrix(1, \([[:alnum:]]\+\).*/\1/p' $robot_KCsymb_pfad`
    robot_KCP=`tr -d "\n" < $robot_KCsymb_pfad | sed -n -e 's/.*kc_symbols := Matrix(1, \([[:alnum:]]\+\), \[\[\(.*\)\]\]);/\2/p' | sed 's/,/ /g'`
  else
    # von Maple wurde noch nichts generiert (passiert beim ersten Start der Skripte, bevor das Maple-Skript für Zwangsbedingungen aufgerufen wurde).
    robot_NKCP="UNDEFINED"
    robot_KCP="UNDEFINED"
  fi;
else
  # Keine ZB definiert
  robot_NKCP=0
fi;

echo "robot_kinconstr_exist=$robot_kinconstr_exist" >> $robot_env_pfad.sh
echo "robot_NKCP=$robot_NKCP" >> $robot_env_pfad.sh
echo "robot_KCP=\"$robot_KCP\"" >> $robot_env_pfad.sh

# Dimension des MPV (aus exportiertem Code)
mpv_fixb_pfad=$repo_pfad/codeexport/${robot_name}/minimal_parameter_vector_fixb_maple
if [ -f $mpv_fixb_pfad ]; then
  # Ersetze Text links und rechts von der Dimension mit nichts.
  robot_NMPVFIXB=`grep "Matrix" $mpv_fixb_pfad | tail -1 | sed 's/.*Matrix[(]\(.*\)/\1/' | sed 's/, 1, .*//'`
else
  robot_NMPVFIXB="NOTDEFINED"
fi
echo "robot_NMPVFIXB=$robot_NMPVFIXB" >> $robot_env_pfad.sh
echo "robot_NMPVFIXB=$robot_NMPVFIXB"

mpv_floatb_pfad=$repo_pfad/codeexport/${robot_name}/minimal_parameter_vector_floatb_eulangrpy_maple
if [ -f $mpv_floatb_pfad ]; then
  robot_NMPVFLOATB=`grep "Matrix" $mpv_floatb_pfad | tail -1 | sed 's/.*Matrix[(]\(.*\)/\1/' | sed 's/, 1, .*//'`
else
  robot_NMPVFLOATB="NOTDEFINED"
fi
echo "robot_NMPVFLOATB=$robot_NMPVFLOATB" >> $robot_env_pfad.sh
echo "robot_NMPVFLOATB=$robot_NMPVFLOATB"


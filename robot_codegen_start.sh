#!/bin/bash
# Starte die komplette Code-Erstellung für eine beliebige Roboterstrukter
#
# Argumente:
# -p, --parallel
#   Parallele Berechnung der Maple-Arbeitsblätter
# --fixb_only
#   Nur Berechnung der Fixed-Base Funktionen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)

# Standard-Einstellungen
CG_PARALLEL=0
CG_FIXBONLY=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    --help)
    echo "zulässige Parameter sind:"
    echo "-p (--parallel): Parallele Berechnung der Maple-Arbeitsblätter"
    echo "--fixb_only: Nur Berechnung der Fixed-Based-Funktionen (nicht: Floating Base)"
    exit 0
    ;;
    -p|--parallel)
    CG_PARALLEL=1
    ;;
    --fixb_only)
    CG_FIXBONLY=1
    ;;
    *)
    echo "Unbekannte Option: $key"
    exit 1
    ;;
esac
shift # past argument or value
done

echo CG_PARALLEL      = "${CG_PARALLEL}"
echo CG_FIXBONLY      = "${CG_FIXBONLY}"

# Argument bestimmen, was an Maple-Ausführungs-Skript angehängt wird.
if [ "$CG_FIXBONLY" == "1" ]; then
  CG_FIXBONLY_ARGUMENT="--fixb_only"
else
  CG_FIXBONLY_ARGUMENT=""
fi;

# Maple-Skripte starten
cd $repo_pfad/robot_codegen_scripts/
if [ "$CG_PARALLEL" == "1" ]; then
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch_par.sh $CG_FIXBONLY_ARGUMENT
else
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch.sh $CG_FIXBONLY_ARGUMENT
fi;

# Matlab-Funktionen generieren
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar.sh

# Testfunktionen generieren
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/testfunctions_generate.sh

# Matlab-Testfunktionen starten
if [ ! "$CG_FIXBONLY" == "1" ]; then
  matlab -nodesktop -nosplash -r "run('$repo_pfad/robot_codegen_testfunctions/${robot_name}_test_everything');quit;"
fi;

if [ "$CG_FIXBONLY" == "1" ]; then
  echo "Funktionsgenerierung abgeschlossen. Gesamt-Test aller Matlab-Funktionen noch nicht implementiert. Für Einzeltests, siehe robot_codegen_testfunctions/${robot_name}_test_everything.m"
else
  echo "Funktionsgenerierung abgeschlossen. Alle Tests erfolgreich."
fi;

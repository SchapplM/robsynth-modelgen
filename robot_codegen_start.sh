#!/bin/bash -e
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
CG_FLOATBONLY=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    --help)
    echo "zulässige Parameter sind:"
    echo "-p (--parallel): Parallele Berechnung der Maple-Arbeitsblätter"
    echo "--fixb_only: Nur Berechnung der Fixed-Base-Funktionen (nicht: Floating Base)"
    echo "--floatb_only: Nur Berechnung der Floating-Base-Funktionen (nicht: Fixed Base)"
    echo "Die letzten beiden Optionen sind exklusiv (nur eine ist möglich)"
    exit 0
    ;;
    -p|--parallel)
    CG_PARALLEL=1
    ;;
    --fixb_only)
    CG_FIXBONLY=1
    ;;
    --floatb_only)
    CG_FLOATBONLY=1
    ;;
    *)
    echo "Unbekannte Option: $key"
    ./robot_codegen_start.sh --help
    exit 1
    ;;
esac
shift # past argument or value
done

echo CG_PARALLEL      = "${CG_PARALLEL}"
echo CG_FIXBONLY      = "${CG_FIXBONLY}"
echo CG_FLOATBONLY      = "${CG_FLOATBONLY}"

if [ "$CG_FIXBONLY" == "1" ] && [ "$CG_FLOATBONLY" == "1" ]; then
  echo "Nicht beide Optionen gleichzeitig möglich: fixb_only, floatb_only"
  exit 1
fi;

# Argument bestimmen, was an Maple-Ausführungs-Skript angehängt wird.
if [ "$CG_FIXBONLY" == "1" ]; then
  CG_BASE_ARGUMENT="--fixb_only"
elif [ "$CG_FLOATBONLY" == "1" ]; then
  CG_BASE_ARGUMENT="--floatb_only"
else
  CG_BASE_ARGUMENT=""
fi;

cd $repo_pfad/robot_codegen_scripts/

# Ordner vorbereiten
source robot_codegen_tmpvar_bash.sh > /dev/null # enthält zunächst unvollständige Definitionen und wird nur für den Roboternamen gebraucht.
mkdir -p "$repo_pfad/codeexport/$robot_name"
mkdir -p "$repo_pfad/codeexport/matlabfcn/$robot_name"
mkdir -p "$repo_pfad/codeexport/testfcn/$robot_name"

# Maple-Definitionen einmal ausführen (damit dort definierte Variablen in Bash übernommen werden)
pwd_alt=$(pwd)
cd /opt/maple2017/bin
nice -n 10 ./maple -q  <<< "currentdir(\"$repo_pfad/robot_codegen_definitions\"): read \"robot_tree_floatb_twist_definitions.mpl\";"
cd $pwd_alt

# Umgebungsvariablen vorbereiten (jetzt enthalten sie die vollen MDH-Informationen (Name, Dimensionen)
source robot_codegen_tmpvar_bash.sh > /dev/null

# Skripte vorbereiten
source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_preparation.sh

# Maple-Skripte starten
if [ "$CG_PARALLEL" == "1" ]; then
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch_par.sh $CG_BASE_ARGUMENT
else
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch.sh $CG_BASE_ARGUMENT
fi;

# Hilfs-Skripte für die Matlab-Code-Generierung
pwd_alt=$(pwd)
cd /opt/maple2017/bin
nice -n 10 ./maple -q  <<< "currentdir(\"$repo_pfad/helper\"): read \"robot_gen_symmat2vector.mpl\";"
cd $pwd_alt

# Matlab-Funktionen generieren
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar.sh

# Testfunktionen generieren
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/testfunctions_generate.sh

# Matlab-Testfunktionen starten
if [ ! "$CG_FIXBONLY" == "1" ]; then
  matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/testfcn/${robot_name}/${robot_name}_test_everything');quit;"
else
  matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/testfcn/${robot_name}/${robot_name}_test_everything_fixbase');quit;"  
fi;

echo "Funktionsgenerierung abgeschlossen. Alle Tests erfolgreich."


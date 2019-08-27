#!/bin/bash -e
# Starte die komplette Code-Erstellung für eine beliebige Roboterstrukter
#
# Argumente:
# -p, --parallel
#   Parallele Berechnung der Maple-Arbeitsblätter
# --minimal
#   Nur die wesentlichen Dynamik- und Kinematikfunktionen erstellen (Kein Regressor, Parametersatz 1)
# --fixb_only
#   Nur Berechnung der Fixed-Base Funktionen.
# --ic
#   Berechne Roboter mit impliziten Zwangsbedingungen
# --parrob
#   Es handelt sich um einen parallelen Roboter
# --not_gen_serial
#   Die Dynamik der seriellen Kette ist bereits vorhanden und soll nicht neu berechnet werden
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)

# Standard-Einstellungen
CG_PARALLEL=0
CG_MINIMAL=0
CG_FIXBONLY=0
CG_FLOATBONLY=0
CG_NOTEST=0
CG_IC=0
CG_PARROB=0
CG_NOTGENSERIAL=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    --help)
    echo "zulässige Parameter sind:"
    echo "-p (--parallel): Parallele Berechnung der Maple-Arbeitsblätter"
    echo "--minimal: Generie nur die Mindestzahl der notwendigen Funktionen"
    echo "--fixb_only: Nur Berechnung der Fixed-Base-Funktionen (nicht: Floating Base)"
    echo "--floatb_only: Nur Berechnung der Floating-Base-Funktionen (nicht: Fixed Base)"
    echo "--ic: Es handelt sich um einen Roboter mit impliziten Zwangsbedingungen"
    echo "--parrob: Es handelt sich um einen parallelen Roboter"
    echo "--not_gen_serial: Neuberechnung serieller/verzweigter Roboter bei parallelem/hybriden Roboter unterbinden"
    echo "--notest: Kein Start der Matlab-Gesamt- und Modultests"
    echo "Die Optionen fixb_only und floatb_only sind exklusiv (nur eine ist möglich)"
    exit 0
    ;;
    -p|--parallel)
    CG_PARALLEL=1
    ;;
    --minimal)
    CG_MINIMAL=1
    ;;
    --notest)
    CG_NOTEST=1
    ;;
    --fixb_only)
    CG_FIXBONLY=1
    ;;
    --floatb_only)
    CG_FLOATBONLY=1
    ;;
    --ic)
    CG_IC=1
    ;;
    --parrob)
    CG_PARROB=1
    ;;
    --not_gen_serial)
    CG_NOTGENSERIAL=1
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
echo CG_MINIMAL       = "${CG_MINIMAL}"
echo CG_FIXBONLY      = "${CG_FIXBONLY}"
echo CG_FLOATBONLY    = "${CG_FLOATBONLY}"
echo CG_IC            = "${CG_IC}"
echo CG_PARROB        = "${CG_PARROB}"
echo CG_NOTGENSERIAL  = "${CG_NOTGENSERIAL}"
echo CG_NOTEST        = "${CG_NOTEST}"

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
if [ "$CG_MINIMAL" == "1" ]; then
  CG_BASE_ARGUMENT="$CG_BASE_ARGUMENT --minimal"
fi;

cd $repo_pfad/robot_codegen_scripts/

### seriellen Roboter berechnen ###
if [ "$CG_NOTGENSERIAL" == "0" ]; then
  # Ordner vorbereiten
  source robot_codegen_tmpvar_bash.sh quiet # enthält zunächst unvollständige Definitionen und wird nur für den Roboternamen gebraucht.
  mkdir -p "$repo_pfad/workdir/tmp"
  mkdir -p "$repo_pfad/codeexport/$robot_name"
  mkdir -p "$repo_pfad/codeexport/$robot_name/tmp"
  mkdir -p "$repo_pfad/codeexport/$robot_name/matlabfcn"
  mkdir -p "$repo_pfad/codeexport/$robot_name/testfcn"

  echo "Beginne Berechnungen für den Roboter ${robot_name}"

  # Maple-Definitionen einmal ausführen (damit dort definierte Variablen in Bash übernommen werden)
  $repo_pfad/scripts/run_maple_script.sh $repo_pfad/robot_codegen_definitions/robot_tree_floatb_twist_definitions.mpl

  # Umgebungsvariablen vorbereiten (jetzt enthalten sie die vollen MDH-Informationen (Name, Dimensionen)
  source robot_codegen_tmpvar_bash.sh > /dev/null

  # Skripte vorbereiten
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_preparation.sh $CG_BASE_ARGUMENT

  # Maple-Skripte starten
  if [ "$CG_PARALLEL" == "1" ]; then
    source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch_par.sh $CG_BASE_ARGUMENT
  else
    source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch.sh $CG_BASE_ARGUMENT
  fi;

  #source robot_codegen_tmpvar_bash.sh

  # Matlab-Funktionen generieren
  cd $repo_pfad/robot_codegen_scripts/
  source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar.sh

  # Testfunktionen generieren
  cd $repo_pfad/robot_codegen_scripts/
  source $repo_pfad/robot_codegen_scripts/testfunctions_generate.sh

  if [ "$CG_MINIMAL" == "0" ] && [ "$CG_NOTEST" != "1" ]; then
   #Matlab-Testfunktionen starten
     if [ ! "$CG_FIXBONLY" == "1" ]; then
       matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/${robot_name}/testfcn/${robot_name}_test_everything');quit;"
     else
       matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/${robot_name}/testfcn/${robot_name}_test_everything_fixbase');quit;"
     fi;
     echo "Funktionsgenerierung abgeschlossen. Alle Tests erfolgreich."
   else
     echo "Funktionsgenerierung abgeschlossen. Keine Tests durchgeführt, da nur Minimalversion erstellt wurde."
  fi;
fi;

### Hybriden Roboter mit impliziten Zwangsbedingungen berechnen ###
if [ "$CG_IC" == "1" ]; then
  # Ordner vorbereiten
  source robot_codegen_tmpvar_bash_ic.sh quiet # wird nur für den Roboternamen gebraucht.
  mkdir -p "$repo_pfad/workdir/tmp"
  mkdir -p "$repo_pfad/codeexport/$robot_name"
  mkdir -p "$repo_pfad/codeexport/$robot_name/tmp"
  mkdir -p "$repo_pfad/codeexport/$robot_name/matlabfcn"
  mkdir -p "$repo_pfad/codeexport/$robot_name/testfcn"
  
  echo "Beginne Berechnungen für den hybriden Roboter mit impliziten Zwangsbedingungen ${robot_name}"

  # Skripte vorbereiten
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_preparation.sh

  # Maple-Skripte starten
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch_ic.sh $CG_BASE_ARGUMENT

  # Matlab-Funktionen generieren
  cd $repo_pfad/robot_codegen_scripts/
  source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar_ic.sh

  # Testfunktionen generieren
  cd $repo_pfad/robot_codegen_scripts/
  source $repo_pfad/robot_codegen_scripts/testfunctions_generate_ic.sh

  # Matlab-Testfunktionen starten
  if [ "$CG_NOTEST" != "1" ]; then
    matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/${robot_name}/testfcn/${robot_name}_test_everything');quit;"
  fi;
  echo "Funktionsgenerierung abgeschlossen. Alle Tests erfolgreich."
fi;

### parallelen Roboter berechnen ###
if [ "$CG_PARROB" == "1" ]; then
  # Ordner vorbereiten
  source robot_codegen_tmpvar_bash_par.sh quiet # enthält zunächst unvollständige Definitionen und wird nur für den Roboternamen gebraucht.
  mkdir -p "$repo_pfad/workdir/tmp"
  mkdir -p "$repo_pfad/codeexport/$robot_name"
  mkdir -p "$repo_pfad/codeexport/$robot_name/tmp"
  mkdir -p "$repo_pfad/codeexport/$robot_name/matlabfcn"
  mkdir -p "$repo_pfad/codeexport/$robot_name/testfcn"

  echo "Beginne Berechnungen für den parallele Roboter ${robot_name}"

  # Maple-Definitionen einmal ausführen (damit dort definierte Variablen in Bash übernommen werden)
  $repo_pfad/scripts/run_maple_script.sh $repo_pfad/robot_codegen_parallel/robot_para_definitions.mpl

  # Umgebungsvariablen vorbereiten (jetzt enthalten sie die vollen MDH-Informationen (Name, Dimensionen)
  source robot_codegen_tmpvar_bash_par.sh > /dev/null

  # Skripte vorbereiten
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_preparation_par.sh

  # Maple-Skripte starten
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch_parrob.sh $CG_BASE_ARGUMENT

  # Matlab-Funktionen generieren
  cd $repo_pfad/robot_codegen_scripts/
  source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar_par.sh

  # Testfunktionen generieren
  cd $repo_pfad/robot_codegen_scripts/
  source $repo_pfad/robot_codegen_scripts/testfunctions_generate_par.sh

  # Matlab-Testfunktionen starten
  if [ "$CG_NOTEST" != "1" ]; then
    matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/${robot_name}/testfcn/${robot_name}_test_everything_par');quit;"
  fi;
  echo "Funktionsgenerierung abgeschlossen. Alle Tests erfolgreich."
fi;

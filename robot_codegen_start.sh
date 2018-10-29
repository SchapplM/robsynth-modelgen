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
    echo "Die letzten beiden Optionen sind exklusiv (nur eine ist möglich)"
    exit 0
    ;;
    -p|--parallel)
    CG_PARALLEL=1
    ;;
    --minimal)
    CG_MINIMAL=1
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
echo CG_MINIMAL       = "${CG_MINIMAL}"
echo CG_FIXBONLY      = "${CG_FIXBONLY}"
echo CG_FLOATBONLY    = "${CG_FLOATBONLY}"

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

# Ordner vorbereiten
source robot_codegen_tmpvar_bash.sh quiet # enthält zunächst unvollständige Definitionen und wird nur für den Roboternamen gebraucht.
mkdir -p "$repo_pfad/workdir/tmp"
mkdir -p "$repo_pfad/codeexport/$robot_name"
mkdir -p "$repo_pfad/codeexport/$robot_name/tmp"
mkdir -p "$repo_pfad/codeexport/$robot_name/matlabfcn"
mkdir -p "$repo_pfad/codeexport/$robot_name/testfcn"

# Handelt es sich um einen parallelen Roboter, so lade das Bein des Roboters und starte das Skript von vorne
robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env
parallel_robot=`grep "parallel := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
parallel_leg_name=`grep "leg_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
echo "parallel_robot=$parallel_robot" >> $robot_env_pfad.sh


if [ "$parallel_robot" == "1" ]; then 
	if [ ! -f $repo_pfad/codeexport/${parallel_leg_name}/tmp/inertia_par2_maple.m ]; then
		if [ ! -f $repo_pfad/codeexport/${parallel_leg_name}/tmp/tree_floatb_definitions ]; then
			cd $repo_pfad/robot_codegen_definitions/
			mv robot_env "robot_env_${robot_name}"
			cd $repo_pfad/robot_codegen_definitions/examples/
			cp "robot_env_${parallel_leg_name}.example" $repo_pfad/robot_codegen_definitions/
			cd $repo_pfad/robot_codegen_definitions/
			mv "robot_env_${parallel_leg_name}.example" robot_env
			echo "Starte Berechnung für die Beine ${parallel_leg_name} der PKM ${robot_name}."
			cd $repo_pfad
			./robot_codegen_start.sh --fixb_only
			echo "Starte Berechnung für die Dynamik der PKM"
			cd $repo_pfad/robot_codegen_definitions/
			rm robot_env
			mv "robot_env_${robot_name}" robot_env
			cd $repo_pfad/robot_codegen_scripts/
		fi;
	else
		# Die Berechnungen für die Beine sind bereits vorhanden -> Lade nur dessen Definitionen
		robot_name_pkm=$robot_name
		cd $repo_pfad/robot_codegen_definitions/
		mv robot_env "robot_env_${robot_name}"
		cd $repo_pfad/robot_codegen_definitions/examples/
		cp "robot_env_${parallel_leg_name}.example" $repo_pfad/robot_codegen_definitions/
		cd $repo_pfad/robot_codegen_definitions/
		mv "robot_env_${parallel_leg_name}.example" robot_env
		echo "Die Berechnungen für die Beine sind bereits vorhanden -> Hole Parameter für die Beine ${parallel_leg_name} der PKM ${robot_name}."
		cd $repo_pfad/robot_codegen_scripts
		$repo_pfad/scripts/run_maple_script.sh $repo_pfad/robot_codegen_definitions/robot_tree_floatb_twist_definitions.mpl
		echo $qJ_t
		source robot_codegen_tmpvar_bash.sh > /dev/null
		cd $repo_pfad/robot_codegen_definitions/
		rm robot_env
		mv "robot_env_${robot_name_pkm}" robot_env
		cd $repo_pfad/robot_codegen_scripts/
		robot_env_pfad=$repo_pfad/robot_codegen_definitions/robot_env
		parallel_robot=`grep "parallel := " $robot_env_pfad | tail -1 | sed 's/.*= \(.*\):/\1/'`
		parallel_leg_name=`grep "leg_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
		robot_name=`grep "robot_name := " $robot_env_pfad | tail -1 | sed 's/.*= "\(.*\)":/\1/'`
		echo "parallel_robot=$parallel_robot" >> $robot_env_pfad.sh
	fi;
fi;

# Maple-Definitionen einmal ausführen (damit dort definierte Variablen in Bash übernommen werden)
if [ "$parallel_robot" == "1" ]; then 
	$repo_pfad/scripts/run_maple_script.sh $repo_pfad/robot_codegen_parallel/robot_para_definitions.mpl
else
	$repo_pfad/scripts/run_maple_script.sh $repo_pfad/robot_codegen_definitions/robot_tree_floatb_twist_definitions.mpl
	echo $qJ_t
fi;

# Umgebungsvariablen vorbereiten (jetzt enthalten sie die vollen MDH-Informationen (Name, Dimensionen)
if [ "$parallel_robot" == "1" ]; then 
	source robot_codegen_tmpvar_pkm_bash.sh > /dev/null
else
	source robot_codegen_tmpvar_bash.sh > /dev/null
fi;

# Skripte vorbereiten
source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_preparation.sh $CG_BASE_ARGUMENT

# Maple-Skripte starten
if [ "$CG_PARALLEL" == "1" ]; then
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch_par.sh $CG_BASE_ARGUMENT
else
  source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch.sh $CG_BASE_ARGUMENT
fi;

# Matlab-Funktionen generieren
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar.sh

# Testfunktionen generieren
if [ "$parallel_robot" == "0" ]; then
	cd $repo_pfad/robot_codegen_scripts/
	source $repo_pfad/robot_codegen_scripts/testfunctions_generate.sh
fi;

if [ "$CG_MINIMAL" == "0" ]; then
  # Matlab-Testfunktionen starten
  if [ ! "$CG_FIXBONLY" == "1" ]; then
    matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/${robot_name}/testfcn/${robot_name}_test_everything');quit;"
  else
    matlab -nodesktop -nosplash -r "run('$repo_pfad/codeexport/${robot_name}/testfcn/${robot_name}_test_everything_fixbase');quit;"  
  fi;
  echo "Funktionsgenerierung abgeschlossen. Alle Tests erfolgreich."
else
  echo "Funktionsgenerierung abgeschlossen. Keine Tests durchgeführt, da nur Minimalversion erstellt wurde."
fi;